//
//
//

#ifndef _TP_LIB_H_
#define _TP_LIB_H_

#include "Eigen/Core"
#include "Eigen/Geometry"

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <vector>
using namespace std;

namespace tp{

enum cost{FREE=0, SUSPICION=50, LETHAL=100};

void
localToGlobalPath(nav_msgs::Path& out_path,
			const nav_msgs::Path& in_path, const	geometry_msgs::Transform trans){

	out_path.poses.clear();
	
	Eigen::Vector3f base(trans.translation.x,trans.translation.y, trans.translation.z);
	Eigen::Quaternion<float> qoat(trans.rotation.w, trans.rotation.x, trans.rotation.y, trans.rotation.z);

	vector<geometry_msgs::PoseStamped>::const_iterator itr;
	for(itr=in_path.poses.begin(); itr!=in_path.poses.end(); itr++){
		Eigen::Vector3f xf, xt(itr->pose.position.x,itr->pose.position.y,itr->pose.position.z);
		xf=qoat.toRotationMatrix()*xt+base;
		
		geometry_msgs::PoseStamped ps;
		ps.pose.position.x=xf[0];
		ps.pose.position.y=xf[1];
		ps.pose.position.z=xf[2];	
		
		out_path.poses.push_back(ps);
	}	
	
	out_path.header=in_path.header;
}

void localPointClooudToGlobal(const sensor_msgs::PointCloud& points_in, const geometry_msgs::Transform& trans, sensor_msgs::PointCloud& points_out){
	points_out=points_in;
	points_out.points.clear();
	
	Eigen::Vector3f base_laser(trans.translation.x,trans.translation.y, trans.translation.z);
	Eigen::Quaternion<float> qoat(trans.rotation.w, trans.rotation.x, trans.rotation.y, trans.rotation.z);
	
	vector<geometry_msgs::Point32>::const_iterator itr;
	for(itr=points_in.points.begin(); itr!=points_in.points.end(); itr++){
		//gpoints.points.push_back(*itr);
		Eigen::Vector3f xf, xt(itr->x,itr->y,itr->z);
		xf=qoat.toRotationMatrix()*xt+base_laser;
		//xf=xt;
		//xf=xt+base_laser;
		geometry_msgs::Point32 pt;
		pt.x=xf[0], pt.y=xf[1], pt.z=xf[2];
		points_out.points.push_back(pt);
	
	}
}


void bresenhamsLine(vector<float>& x, vector<float>& y, const vector<geometry_msgs::PoseStamped>& path, float resolution=0.15){
	
	int length=path.size();

	for(int i=0; i<length-2; i++){
		float x1=path[i].pose.position.x;
		float y1=path[i].pose.position.y;
		float x2=path[i+1].pose.position.x;
		float y2=path[i+1].pose.position.y;

		bool steep= fabs(y2-y1)>fabs(x2-x1);
		
		if(steep){
			swap(x1,y1);
			swap(x2,y2);
		}
		if(x1>x2){
			swap(x1,x2);
			swap(y1,y2);		
		}
		
		float deltax=x2-x1;
		float deltay=fabs(y2-y1);	
		float error=0;
		float deltaerr=deltay/deltax;	
		float ystep;
		float yt=y1;
		if(y1<y2){
			ystep=resolution;
		}else{
			ystep=-resolution;
		}
		for(float xt=x1; xt<=x2; xt+=resolution){
			if(steep){
				x.push_back(yt);
				y.push_back(xt);
			}else{
				x.push_back(xt);
				y.push_back(yt);				
			}
			
			error+=deltaerr;
			
			if(error>=0.5){
				yt+=ystep;
				error-=1;
			}
			
		}	
		
	}
}

float yawFromQuat(const Eigen::Quaternion<float>& quat){
	Eigen::Matrix3f mat=quat.toRotationMatrix();
	float theta_ret=atan2(mat(1,0),mat(0,0));
	return theta_ret;
}


double piToPI(double theta){
	double ret=theta;
	
	if(theta>M_PI){
		ret=theta-M_PI;
	}

	if(theta<-M_PI){
		ret=theta+M_PI;
	}
	
	return ret;
}


void getGridCells(nav_msgs::GridCells& cells, const nav_msgs::OccupancyGrid local_map){
	cells.cells.clear();
	
	cells.header.frame_id=local_map.header.frame_id;
	cells.header.stamp=local_map.header.stamp;
	cells.cell_width=local_map.info.resolution;
	cells.cell_height=local_map.info.resolution;
	
	
	for(int xi=0; xi<(int)local_map.info.height; xi++){
		for(int yi=0; yi<(int)local_map.info.width; yi++){
			if(local_map.data[xi+local_map.info.width*yi]!=FREE){
				float x=local_map.info.origin.position.x-xi*local_map.info.resolution;
				float y=local_map.info.origin.position.y-yi*local_map.info.resolution;
				
				geometry_msgs::Point pt;
				pt.x=x;	pt.y=y;	pt.z=0;
				cells.cells.push_back(pt);
			}
		}
	}
}


};


#endif //  _TP_LIB_H_
