#include "trajectory_generation/Visualize_lib.h"
#include <iostream>

using namespace std;

Visualization::Visualization(std_msgs::ColorRGBA rgba_in,
							 uint action_in,
							 float scale_in,
							 const string& name_in,
							 const string& frame_id_in)
{
	marker.color = rgba_in;
	marker.action = action_in;
	marker.scale.x=scale_in;
	marker.scale.y=scale_in;
	marker.scale.z=scale_in;
	marker.ns = name_in;
	marker.header.frame_id = frame_id_in;
}
///////////////////////////////////////////////////////
/////-----------Visualize function-------------////////
///////////////////////////////////////////////////////
void Visualization::convertPath2MarkerLine(	nav_msgs::Path path,
								visualization_msgs::Marker& line, 
								int id)
{
	line = marker;
	line.header.stamp=ros::Time::now();
	line.id = id; // set each ID number of lines
	line.type=visualization_msgs::Marker::LINE_STRIP;

	line.pose.position.x=0.0;
	line.pose.position.y=0.0;
	line.pose.position.z=0.0;
	line.pose.orientation.x = 0.0;
	line.pose.orientation.y = 0.0;
	line.pose.orientation.z = 0.0;
	line.pose.orientation.w = 0.0;

	
	
	geometry_msgs::Point pt;
	for(uint i=0; i<path.poses.size(); i++){
		pt.x = path.poses[i].pose.position.x;
		pt.y = path.poses[i].pose.position.y;
		pt.z = path.poses[i].pose.position.z-0.1;
		line.points.push_back(pt);
	}
}

void Visualization::localToGlobal(const geometry_msgs::Point& pose_in, 
					const geometry_msgs::PoseStamped& robot, 
					geometry_msgs::Point& pose_out)
{
	pose_out=pose_in;
	//float angle = tf::getYaw(trans.getRotation());
	//geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,angle);
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,robot.pose.orientation.w);
	
	Eigen::Vector3f base_laser(
		robot.pose.position.x,
		robot.pose.position.y,
		robot.pose.position.z
		);
	/*Eigen::Quaternion<float> qoat(
		robot.pose.orientation.w, 
		robot.pose.orientation.x, 
		robot.pose.orientation.y, 
		robot.pose.orientation.z
		);*/
	Eigen::Quaternion<float> qoat(
		odom_quat.w, 
		odom_quat.x, 
		odom_quat.y, 
		odom_quat.z
		);
	
	//vector<geometry_msgs::Point32>::const_iterator itr;
	//for(itr=points_in.points.begin(); itr!=points_in.points.end(); itr++){
		//Eigen::Vector3f xf, xt(itr->x,itr->y,itr->z);
		Eigen::Vector3f xf, xt(pose_in.x,pose_in.y,pose_in.z);
		xf = qoat.toRotationMatrix()*xt + base_laser;
		pose_out.x=xf[0], pose_out.y=xf[1], pose_out.z=xf[2];
		//geometry_msgs::Point32 pt;
		//pt.x=xf[0], pt.y=xf[1], pt.z=xf[2];
		//points_out.points.push_back(pt);
	
	//}
}

///////////////////////////////////////////////////////
/////-----------Visualize function-------------////////
///////////////////////////////////////////////////////
void Visualization::convertPath2MarkerFoot(	nav_msgs::Path path,
										visualization_msgs::Marker& line, 
										float tate,float yoko,
										int id)
{
	line = marker;
	line.header.stamp=ros::Time::now();
	line.id = id; // set each ID number of lines
	line.type=visualization_msgs::Marker::LINE_LIST;

	line.pose.position.x=0.0;
	line.pose.position.y=0.0;
	line.pose.position.z=0.0;
	line.pose.orientation.x = 0.0;
	line.pose.orientation.y = 0.0;
	line.pose.orientation.z = 0.0;
	line.pose.orientation.w = 0.0;

	
	geometry_msgs::Point pose_in11;
	geometry_msgs::Point pose_in21;
	geometry_msgs::Point pose_in31;
	geometry_msgs::Point pose_in41;
	geometry_msgs::Point pose_in12;
	geometry_msgs::Point pose_in22;
	geometry_msgs::Point pose_in32;
	geometry_msgs::Point pose_in42;
	
	float half_tate = tate/2;
	float half_yoko = yoko/2;
	
	pose_in11.x = half_tate; pose_in11.y = -half_yoko; pose_in11.z = 0.01;
	
	pose_in12.x = half_tate; pose_in12.y = half_yoko; pose_in12.z = 0.01;
	pose_in21.x = half_tate; pose_in21.y = half_yoko; pose_in21.z = 0.01;
	
	pose_in22.x = -half_tate; pose_in22.y = half_yoko; pose_in22.z = 0.01;
	pose_in31.x = -half_tate; pose_in31.y = half_yoko; pose_in31.z = 0.01;
	
	pose_in32.x = -half_tate; pose_in32.y = -half_yoko; pose_in32.z = 0.01;
	pose_in41.x = -half_tate; pose_in41.y = -half_yoko; pose_in41.z = 0.01;
	
	pose_in42.x = half_tate; pose_in42.y = -half_yoko; pose_in42.z = 0.01;
	/*geometry_msgs::Point pt;
	for(uint i=0; i<path.poses.size(); i++){
		pt.x = path.poses[i].pose.position.x;
		pt.y = path.poses[i].pose.position.y;
		pt.z = path.poses[i].pose.position.z-0.1;
		line.points.push_back(pt);
	}*/
	for(uint i=0; i<path.poses.size(); i+=5){
		geometry_msgs::Point pose_out11;
		geometry_msgs::Point pose_out21;
		geometry_msgs::Point pose_out31;
		geometry_msgs::Point pose_out41;
		geometry_msgs::Point pose_out12;
		geometry_msgs::Point pose_out22;
		geometry_msgs::Point pose_out32;
		geometry_msgs::Point pose_out42;
		localToGlobal(pose_in11, path.poses[i], pose_out11);
		localToGlobal(pose_in12, path.poses[i], pose_out12);
		localToGlobal(pose_in21, path.poses[i], pose_out21);
		localToGlobal(pose_in22, path.poses[i], pose_out22);
		localToGlobal(pose_in31, path.poses[i], pose_out31);
		localToGlobal(pose_in32, path.poses[i], pose_out32);
		localToGlobal(pose_in41, path.poses[i], pose_out41);
		localToGlobal(pose_in42, path.poses[i], pose_out42);
		line.points.push_back(pose_out11);
		line.points.push_back(pose_out12);
		line.points.push_back(pose_out21);
		line.points.push_back(pose_out22);
		line.points.push_back(pose_out31);
		line.points.push_back(pose_out32);
		line.points.push_back(pose_out41);
		line.points.push_back(pose_out42);
	}
}

void Visualization::convertPath2MarkerLineOffset(nav_msgs::Path path,
													visualization_msgs::Marker& line, 
													int id, float offset)
{
	line = marker;
	line.header.stamp=ros::Time::now();
	line.id = id; // set each ID number of lines
	line.type=visualization_msgs::Marker::LINE_STRIP;

	line.pose.position.x=0.0;
	line.pose.position.y=0.0;
	line.pose.position.z=0.0;
	line.pose.orientation.x = 0.0;
	line.pose.orientation.y = 0.0;
	line.pose.orientation.z = 0.0;
	line.pose.orientation.w = 0.0;

	
	
	geometry_msgs::Point pt;
	for(uint i=0; i<path.poses.size(); i++){
		pt.x = path.poses[i].pose.position.x;
		pt.y = path.poses[i].pose.position.y;
		pt.z = path.poses[i].pose.position.z + offset;
		line.points.push_back(pt);
	}
}

void Visualization::txtMarker(	const string& txt_in,
								geometry_msgs::PoseStamped goal,
								visualization_msgs::Marker& mk, 
								int id)
{
	mk = marker;
	mk.header.stamp=ros::Time::now();

	mk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    mk.id = id;
	mk.text = txt_in;
	mk.pose = goal.pose;
	mk.scale.x = 0.1;
	mk.scale.y = 0.1;
	mk.scale.z = 0.1;
	mk.color.r = 0.9;
	mk.color.g = 0.9;
	mk.color.b = 0.9;
	mk.color.a = 0.9;
	mk.lifetime = ros::Duration();
}
