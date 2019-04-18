#include "ros/ros.h"
#include <cstdlib>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

using namespace std;


#define SLOW 0
#define NORMAL 1
#define FAST 2
//last up date 2014.11.07.21:00 written by masada


//const float vv = 0.65;//speed
const float vv = 1.00;//speed
const float Vmin = 0.30;
const float Vmax = 1.35;
const float Lmin = 3.0;//previous 3.0
const float Lmax = 5.5;


int wp_mode=0;
boost::mutex wayp_mutex_,speed_mutex_, odo_mutex;
std_msgs::Int32 wayp_;
std_msgs::Float32 speed_ratio_;
nav_msgs::Odometry g_odo; 
bool odo_flag=false;

float speedDecision()
{
	// std_msgs::Int32 wayp;
	// {
		// boost::mutex::scoped_lock(wayp_mutex_);
		// wayp=wayp_;
	// }
	
	std_msgs::Float32 speed_ratio;	
	// {
		// boost::mutex::scoped_lock(speed_mutex_);
		// speed_ratio=speed_ratio_;
	// }
	    
	switch (wp_mode){
		case SLOW:
			//speed_ratio.data=0.4;
			speed_ratio.data=0.7;
			break;
		case NORMAL:
			speed_ratio.data=0.7;
			break;
		case FAST:
			speed_ratio.data=0.7;
			break;
		default:
			speed_ratio.data=0.7;
			break;
	}
    return vv*speed_ratio.data;
}

///////////////////////////////////////////////////////
/////---------Trajectory Length Control--------////////
///////////////////////////////////////////////////////
float trajectoryLengthControl(float vt)
{
    //keep the speed within the set range
	if(vt < Vmin){
		vt = Vmin;
	}
	if(vt > Vmax){
		vt = Vmax;
	}
    //(vt-Vmin)/(Vmax-Vmin) means normalization
	float L = (Lmax - Lmin) * (vt - Vmin)/(Vmax - Vmin) + Lmin;
	return L;
}

void tinyCallback(const nav_msgs::OdometryConstPtr& msg)
{
	g_odo=*msg; 
	odo_flag = true;
}

void waypCallback(const std_msgs::Int32ConstPtr& msg){
	boost::mutex::scoped_lock(wayp_mutex_);	
	wayp_=*msg;
}

void speedCallback(const std_msgs::Float32ConstPtr& msg){
	boost::mutex::scoped_lock(speed_mutex_);	
	speed_ratio_=*msg;
}

void WpModeCallback(const std_msgs::Int32ConstPtr& msg)////
{
	wp_mode=msg->data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "speed_control");
	
    ros::NodeHandle n;
	ros::Subscriber sub_wayp = n.subscribe("/waypoint/now", 100, waypCallback);	
	ros::Subscriber sub_speed_ratio = n.subscribe("/plan/speed_ration", 100, speedCallback);	
	ros::Subscriber wp_mode_sub = n.subscribe("/speed_mode", 1, WpModeCallback);///
	ros::Subscriber tiny_sub = n.subscribe("/tinypower/odom", 1, tinyCallback);
    //Setting the velocity depending on waypoint 
	ros::Publisher pub1 = n.advertise<std_msgs::Float32>("/control/velo", 100);
    //r means the radius of the local path
	ros::Publisher pub2 = n.advertise<std_msgs::Float32>("/control/r", 100);
	
	
	ros::Rate loop_rate(10);
	ros::Time pre_time=ros::Time::now();
	while(ros::ok()){
		
		std_msgs::Float32 velo, r;
		velo.data = speedDecision();
		r.data = trajectoryLengthControl(velo.data);
		
		//cout<<"velo = "<<velo.data<<"\tr = "<<r.data<<endl;
		pub1.publish(velo);
		pub2.publish(r);
		
		odo_flag=false;
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	ROS_INFO("killing now!!!!!");
	return 0;
}
