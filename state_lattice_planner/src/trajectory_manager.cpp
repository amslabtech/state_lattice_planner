#include "ros/ros.h"
#include <trajectory_generation/TrajectoryGeneration.h>
#include <trajectory_generation/Velocity.h>
#include <trajectory_generation/VelocityArray.h>
#include <cstdlib>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
//---------------From library---------------//
#include "trajectory_generation/Visualize_lib.h"

#define THRESH_N_DIST 0.35
#define TRACE 1
using namespace std;

const string header_frame("/base_link");
const string robot_frame("/velodyne");

//callback mutex
boost::mutex l_goal_mutex_;

//to use callback msg in main function
geometry_msgs::PoseStamped g_l_goal; 
nav_msgs::Odometry g_odo; 
std_msgs::Float32 g_radius;
std_msgs::Float32 g_speed; 

boost::mutex res_mutex_;
trajectory_generation::TrajectoryGeneration::Response res_path;
boost::mutex req_mutex_;
trajectory_generation::TrajectoryGeneration::Request req_path;

bool l_goal_flag = false, odometry_flag=false;

inline double calcDistance(double dx, double dy)
{
	return sqrt(dx*dx+dy*dy);
}

geometry_msgs::Pose2D nearest(	geometry_msgs::Pose2D A, 
								geometry_msgs::Pose2D B, 
								geometry_msgs::Pose2D P )
{
	geometry_msgs::Pose2D a, b;
	double r;
	//a = (next wp - now wp),b = (robo pos - now_wp)
	a.x = B.x - A.x;
	a.y = B.y - A.y;
	b.x = P.x - A.x;
	b.y = P.y - A.y;
	
	r = (a.x*b.x + a.y*b.y) / (a.x*a.x + a.y*a.y);
	
	if( r<= 0 ){
		return A;
	}else if( r>=1 ){
		return B;
	}else{
		geometry_msgs::Pose2D result;
		result.x = A.x + r*a.x;
		result.y = A.y + r*a.y;
		return result;
	}
}

bool checkSelectedPath(tf::StampedTransform& transform, trajectory_generation::TrajectoryGeneration& trj)
{
	double n_dist=0;
	geometry_msgs::Pose2D A, B, P;
	A.x=trj.response.path.poses[0].pose.position.x;
	A.y=trj.response.path.poses[0].pose.position.y;
	B.x=trj.response.path.poses[trj.response.path.poses.size()-1].pose.position.x;
	B.y=trj.response.path.poses[trj.response.path.poses.size()-1].pose.position.y;
	for (size_t i=0; i<trj.response.path.poses.size(); ++i){
		P.x=trj.response.path.poses[i].pose.position.x;
		P.y=trj.response.path.poses[i].pose.position.y;
		geometry_msgs::Pose2D H = nearest(A, B, P);
		n_dist=calcDistance(H.x-P.x, H.y-P.y);
		if (n_dist>THRESH_N_DIST){
			cout<<"n_dist:"<<n_dist<<endl;
			return  true;
		}
		
	}
	return false;
}

geometry_msgs::PoseStamped invhomoTrance(geometry_msgs::PoseStamped& p, 
										 geometry_msgs::PoseStamped robot)
{
	geometry_msgs::PoseStamped out;
	float Px = p.pose.position.x - robot.pose.position.x;
	float Py = p.pose.position.y - robot.pose.position.y;
	float p_yaw = tf::getYaw(p.pose.orientation);
	
	float yaw = tf::getYaw(robot.pose.orientation);
	float C = cos(yaw);
	float S = sin(yaw);
	
	out.pose.position.x = Px*C + Py*S;
	out.pose.position.y = Py*C - Px*S;
	out.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,p_yaw-yaw);
	//out.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw+p_yaw);
	return out;
}


/////////////////////////////////////////////////////////
//-------------------Visualize!!!!!--------------------//
/////////////////////////////////////////////////////////
void visualizeInit(visualization_msgs::Marker& arrow)
{
	arrow.header.frame_id=header_frame;
	arrow.header.stamp=ros::Time::now();
	arrow.ns="arrow";
	//arrow.action = visualization_msgs::Marker::ADD;
	arrow.id=0; // set each ID number of arrow
	arrow.type=visualization_msgs::Marker::ARROW;
		
	arrow.pose.position.x=0;
	arrow.pose.position.y=0;
	arrow.pose.position.z=0;
	arrow.pose.orientation.x = 0.0;
	arrow.pose.orientation.y = 0.0;
	arrow.pose.orientation.z = 0.0;
	arrow.pose.orientation.w = 0.0;

	arrow.scale.x=2.8;
	arrow.scale.y=2.8;
	arrow.scale.z=2.8;

	arrow.color.g=0.8;
	arrow.color.r=0.1;
	arrow.color.b=0.0;
	arrow.color.a=1.0;
}

///////////////////////////////////////////////////////
/////-----------Trajectory set mode------------////////
///////////////////////////////////////////////////////
void trjSetMode(trajectory_generation::TrajectoryGeneration& trj, tf::StampedTransform transform)
{
	geometry_msgs::PoseStamped l_goal;
	l_goal=g_l_goal;
	
	trj.request.fin = false;
	trj.request.teledrive = false;
	trj.request.r = g_radius.data;
	trj.request.params.vt = g_speed.data;
	trj.request.params.af = -0.4;
	trj.request.params.vf = g_speed.data;
	trj.request.params.a0 = 0.4;
    trj.request.params.v0 = g_odo.twist.twist.linear.x;
    trj.request.params.k0 = g_odo.twist.twist.angular.z/(g_odo.twist.twist.linear.x+0.0000000001);
    trj.request.start.pose.position.x = transform.getOrigin().x();
    trj.request.start.pose.position.y = transform.getOrigin().y();
    trj.request.start.pose.position.z = 0;
    float angle = tf::getYaw(transform.getRotation());
    trj.request.start.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,angle);
    trj.request.goal = invhomoTrance(l_goal,trj.request.start);
	cout<<"Local goal callbacked !!!!"<<endl;
}

void showPath(nav_msgs::Path& path, ros::Publisher& pub)
{
	std_msgs::ColorRGBA rgba_in;
	rgba_in.r = 1.0;
	rgba_in.g = 0;
	rgba_in.b = 0;
	rgba_in.a = 0.8;
    Visualization* marker_line = new Visualization(rgba_in, ADD,0.1, "OnlyPath",header_frame);
	
	visualization_msgs::Marker line1;
	marker_line->convertPath2MarkerLine(path, line1, 1);
		
	pub.publish(line1);
	delete marker_line;
}

void LocalGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	boost::mutex::scoped_lock(l_goal_mutex_);
	g_l_goal=*msg;
    l_goal_flag=true;
}

void OdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
	g_odo=*msg; 
	odometry_flag = true;
}

void trajectoryManage()
{
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<trajectory_generation::TrajectoryGeneration>("/local_path/local_path");
	ros::Subscriber odom_sub = n.subscribe("/odom", 1, OdomCallback);
	ros::Subscriber l_goal_sub = n.subscribe("/local_goal", 1, LocalGoalCallback);
	
	ros::Publisher visu_path_pub = n.advertise<visualization_msgs::Marker>("/local_path/localpath_vis", 10);
	ros::Publisher vel_arr_pub = n.advertise<trajectory_generation::VelocityArray>("/local_path/velocity_array", 10);
	
    tf::TransformListener listener;
    std_msgs::Bool back_flag;
    back_flag.data=false;

	g_radius.data = 3.14;
	g_speed.data = 0.7;
    
    bool tf_flag=false;
    
    ros::Rate loop_rate(10);
    while(ros::ok()){
        
         tf::StampedTransform transform;
        //Set Robot's state//
        try{
			// listener.waitForTransform(header_frame, robot_frame, ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform(header_frame, robot_frame, ros::Time(0), transform);
            tf_flag=true;
        }
        catch (tf::TransformException ex){
			cout<<"waiting for global and robot frame relation"<<endl;
            // ROS_ERROR("%s",ex.what());
        }
            
        //Set Trajectory configuration//
        if (l_goal_flag && odometry_flag && tf_flag){
         	trajectory_generation::TrajectoryGeneration trj;
            trjSetMode(trj, transform);
            if(trj.request.params.vt > 0){
                if (client.call(trj))	ROS_INFO("Changing Trajectory");
                else  					ROS_ERROR("Failed to call service");
            }else{
                cout<<"Robot speed is ZERO !!!!"<<endl;
            }
            
            //パスの有無を判定
            if(!(trj.response.v_array.vel.size() > 0)){
                cout<<"An trajectory is not generated....."<<endl;
            }else{
                cout<<"An trajectory is generated!!!"<<endl;
                showPath(trj.response.path, visu_path_pub);
            }
            vel_arr_pub.publish(trj.response.v_array);
            
            //l_goal_flag = false;
            //odometry_flag = false;
            tf_flag = false;
        }else{
            cout<<"local_goal_flag:"<<l_goal_flag<<"\todometry_flag:"<<odometry_flag<<"\tr_flag"<< endl;
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "trajectory_manager");
	trajectoryManage();
	return 0;
}
