//
//
//

#ifndef VISUALIZATION_CLASS_H
#define VISUALIZATION_CLASS_H

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/ColorRGBA.h>
#include <boost/thread.hpp>
#include <string>
#include "Eigen/Core"
#include "Eigen/Geometry"
#define ADD		0
#define DELETE	2

using namespace std;

class Visualization{
private:
	Visualization(Visualization&);
	
public:
	Visualization(std_msgs::ColorRGBA rgba_in, uint action_in,float scale_in,const string& name_in, const string& frame_id_in);
	void convertPath2MarkerLine(nav_msgs::Path path,
								visualization_msgs::Marker& line, 
								int id);
	
	void convertPath2MarkerLineOffset(nav_msgs::Path path,
										visualization_msgs::Marker& line, 
										int id, float offset);
	
	void txtMarker(	const string& txt_in,
					geometry_msgs::PoseStamped goal,
					visualization_msgs::Marker& mk, 
					int id);
	
	void convertPath2MarkerFoot(	nav_msgs::Path path,
										visualization_msgs::Marker& line, 
										float tate,float yoko,
										int id);
										
	void localToGlobal(const geometry_msgs::Point& pose_in, 
					const geometry_msgs::PoseStamped& robot, 
					geometry_msgs::Point& pose_out);
	// 管理しているMarker
	visualization_msgs::Marker marker;
};


#endif //LOCAL_MAP_ORGANAIZER

