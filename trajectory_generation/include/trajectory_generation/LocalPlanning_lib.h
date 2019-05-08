//
//
//

#ifndef LOCAL_PLANNING_LOCALPATHPLANNING_CLASS_H
#define LOCAL_PLANNING_LOCALPATHPLANNING_CLASS_H

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include "pathcheck_lib.h"

#include <boost/thread.hpp>
#include <string>
#include <Eigen/Geometry>
using namespace Eigen;
float rad2rad(float rad);
int Sign(float x);
float distance(float x1,float y1,float x2,float y2);
float pathCheck(nav_msgs::Path path, nav_msgs::OccupancyGrid map);
void pathLocal2Global(nav_msgs::Path& path, const tf::StampedTransform transform);

void localToGlobal(	geometry_msgs::Pose local_point,
					geometry_msgs::Pose& global,
					const tf::StampedTransform transform);
float pathMake(const tf::StampedTransform trans, 
			float v, 
			float yawrate, 
			float time, 
			nav_msgs::Path& pa_, 
			const geometry_msgs::PoseStamped endp);

#endif // _TALKER_CLASS_H

