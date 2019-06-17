#include "ros/ros.h"
#include <trajectory_generation/TrajectoryGeneration.h>
#include <trajectory_generation/Velocity.h>
#include <trajectory_generation/VelocityArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

using namespace std;

const float Vmax=0.9;
const float ANGULARmax=1.0; 

ros::Publisher vel_pub;
int time_count=0;
bool v_a_flag=false, joy_flag=false;
geometry_msgs::Twist joy_vel;
boost::mutex v_array_mutex_;
trajectory_generation::VelocityArray g_v_array; 
bool stop_flag=false;

void setStopCommand(geometry_msgs::Twist& cmd_vel){
	cmd_vel.linear.x = 0;
	cmd_vel.angular.z = 0;
}

void commandDecision(trajectory_generation::VelocityArray& v_a,
						geometry_msgs::Twist& cmd_vel){
	float hoge = 15;
	int ind = hoge+time_count;
	if((int)v_a.vel.size() > ind){
		cmd_vel.linear.x = v_a.vel[ind].op_linear;
		cmd_vel.angular.z = v_a.vel[ind].op_angular;
	}else{
		cmd_vel.linear.x = 0;
		cmd_vel.angular.z = 0;
	}
}

void JoyCallback(const geometry_msgs::TwistPtr& msg){
	joy_vel.linear.x = msg->linear.x;
	joy_vel.angular.z = msg->angular.z;
	joy_flag = true;
}

void vArrayCallback(const trajectory_generation::VelocityArrayConstPtr& msg){
	boost::mutex::scoped_lock(v_array_mutex_);
	g_v_array=*msg;
	v_a_flag=true;
}

void StopCallback(const std_msgs::BoolConstPtr& msg){
	stop_flag = msg->data;
}

void MotionPlanner()
{
	ros::NodeHandle n;
	ros::Subscriber joy_sub   = n.subscribe("/joy/vel", 10, JoyCallback);
	ros::Subscriber v_array_sub = n.subscribe("/local_path/velocity_array", 10, vArrayCallback);
	ros::Subscriber stop_sub = n.subscribe("/stop",1,StopCallback);

	vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	
	bool turn_flag = false;
	geometry_msgs::Twist cmd_vel;

	ros::Rate loop_rate(10);
	while(ros::ok()){
		if (!joy_flag && v_a_flag){ //normal state
			trajectory_generation::VelocityArray v_array; 
			boost::mutex::scoped_lock(v_array_mutex_);
			v_array = g_v_array;
			// set velocity and angular //
			if(stop_flag == true){
				cout<<"stop"<<endl;
				setStopCommand(cmd_vel);
			}
			else if(stop_flag == false){ 
				cout<<"normal"<<endl;
				commandDecision(v_array, cmd_vel);
				time_count=0;
			}
			else{
				cout<<"unknown"<<endl;
				setStopCommand(cmd_vel);
				time_count=0;
			}
			// safety //
			if (cmd_vel.linear.x>Vmax) cmd_vel.linear.x=Vmax;
			if (cmd_vel.angular.z>ANGULARmax) cmd_vel.angular.z=ANGULARmax;
			else if (cmd_vel.angular.z<-ANGULARmax) cmd_vel.angular.z=-ANGULARmax;
			
			// publish //
			vel_pub.publish(cmd_vel);
			
			cout<<"stop flag: "<<stop_flag<<endl;
			cout<<"lin = "<<cmd_vel.linear.x<<"\tang = "<<cmd_vel.angular.z<<endl<<endl;
		}
		else if (joy_flag){
			vel_pub.publish(joy_vel);
			joy_flag=false;
		}
		else { //waiting for callback
			cout<<"-----------"<<endl;
			cout<<"v_arr:"<<v_a_flag<<endl;
			cout<<"joy_flag:"<<joy_flag<<endl;
			cout<<"-----------"<<endl;
			cmd_vel.linear.x=0;
			cmd_vel.angular.z=0;
			vel_pub.publish(cmd_vel);
		}
		time_count++;
		
		ros::spinOnce();
		loop_rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motion_decision");
	MotionPlanner();
	
	ROS_INFO("Killing now!!!!!");
	return 0;
}
