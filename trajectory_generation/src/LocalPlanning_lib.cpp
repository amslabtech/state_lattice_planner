#include "trajectory_generation/LocalPlanning_lib.h"
#include <iostream>

using namespace std;

/////////////////////////////////////////////////////////
//-------------------Sign関数!!!!!---------------------//
//---------------引数の符号を返します-------------------//
/////////////////////////////////////////////////////////
int Sign(float x)
{
	int sign = 1;
	if(x < 0)	sign = -1;
	else if(x==0)	sign = 0;
	return sign;
}

float
distance(float x1,float y1,float x2,float y2)
{
	return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}


float rad2rad(float rad)
{
	if(rad > M_PI){
		return rad - 2*M_PI;
	}else if(rad < -M_PI){
		return rad + 2*M_PI;
	}else{
		return rad;
	}
}

///////////////////////////////////////////////////////
//----------Checking collision function--------------//
//bresenhamsLineアルゴリズムによりpathをグリッド化し，
//グリッドマップとの照らし合わせを行います．
//costはpathとGridMapが重なった部分の占有率です.
//占有率 = (0　～　100)
///////////////////////////////////////////////////////
class GridBase{
	private:

		//GridBase(const ExpandMap&);
		
		
	public:
		float resolution;
		uint width;
		uint height;		
		float origin_x;
		float origin_y;
		float origin_yaw;
		Matrix2f rot;
		
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//rot=Rotation2Df(M_PI);
		
		GridBase(nav_msgs::OccupancyGrid gmap)
		{
			resolution = gmap.info.resolution;
			width = gmap.info.width;
			height = gmap.info.height;		
			origin_x = gmap.info.origin.position.x;
			origin_y = gmap.info.origin.position.y;
			origin_yaw = tf::getYaw(gmap.info.origin.orientation);
			float angle = rad2rad(-origin_yaw+M_PI);
			rot = Rotation2Df(angle);
			cout<<"---------------------------------------"<<endl;
			cout<<"gmap.info.origin.position.x="<<gmap.info.origin.position.x<<endl;
			cout<<"gmap.info.origin.position.y="<<gmap.info.origin.position.y<<endl;
			cout<<"orientation x y z w"<<gmap.info.origin.orientation.x<<" "<<gmap.info.origin.orientation.y<<" "<<gmap.info.origin.orientation.z<<" "<<gmap.info.origin.orientation.w<<endl;;
			cout<<"origin_x="<<origin_x<<"\torigin_y="<<origin_y<<endl;
			cout<<"origin_yaw="<<origin_yaw<<"\tangle="<<angle<<endl;
			//cout<<"rot="<<rot<<endl;
			//rot <<	cos(-origin_yaw),	-sin(-origin_yaw),
			//		sin(-origin_yaw),	cos(-origin_yaw);
			/*Eigen::Quaternion<float> qoat(gmap.info.origin.orientation.w,
										   gmap.info.origin.orientation.x,
										   gmap.info.origin.orientation.y,
										   gmap.info.origin.orientation.z);
			rot = qoat.toRotationMatrix();*/
		}
		int Sign(float x)
		{
			int sign = 1;
			if(x < 0)	sign = -1;
			else if(x==0)	sign = 0;
			return sign;
		}
		float Grid(float x)
		{
			return roundf(x / resolution);
		}
		
		int PutX(uint i)
		{
			return roundf(origin_x/resolution) - i;	//for local map
			//return roundf(origin_x/resolution) + i;	//for global map
		}
		int PutY(uint i)
		{
			return roundf(origin_y/resolution) - i;	//for local map
			//return roundf(origin_y/resolution) + i;	//for global map
		}
		/*
		void PutXYwithRot(uint xi, uint yi, int& x_out, int& y_out)
		{
			float x_loc = xi*resolution;
			float y_loc = yi*resolution;
			Eigen::Vector3f conv, xt(x_loc, y_loc, 0);
			conv = rot*xt;
			x_out = Grid(origin_x + conv(0));
			y_out = Grid(origin_y + conv(1));
		}*/
		void globalGridtoLocal(float x_in, float y_in, int& x_out, int& y_out)
		{
			Eigen::VectorXf conv(2), xt(2);
			xt << -x_in+origin_x, -y_in+origin_y;
			conv = rot*xt;
			//conv = xt;
			x_out = Grid( conv(0) );
			y_out = Grid( conv(1) );
		}
		int MapIdx(uint x, uint y)
		{
			return width*y + x;
		}
	
};

float pathCheck(nav_msgs::Path path, nav_msgs::OccupancyGrid map)
{
	
			cout<<"Pathcheck!!!!!"<<endl;
	GridBase* grid_b = new GridBase(map);
	const int expand=0;
	vector<float> x,y;
	tp::bresenhamsLine(x,y, path.poses, map.info.resolution);
	
	float cost=0;
	int count=0;
	int n=x.size();
	for(int i=0; i<n; i++){
		int xi,yi;
		//int xi=round( (map.info.origin.position.x - x[i])/map.info.resolution );
		//int yi=round( (map.info.origin.position.y - y[i])/map.info.resolution );
		grid_b->globalGridtoLocal(x[i],y[i],xi,yi);
		//-----------sawa---------------//
		if(!((97<=xi&&xi<=103)&&(95<=yi&&yi<=104))){
			if(xi>=expand && xi<(int)map.info.height-expand && yi>=expand && yi<(int)map.info.width-expand){
			
				if(map.data[xi+map.info.width*yi] > cost){
					cost = map.data[xi+map.info.width*yi];
				}else if(map.data[xi+expand+map.info.width*(yi+expand)] > cost){
					cost = map.data[xi+expand+map.info.width*(yi+expand)];
				}else if(map.data[xi-expand+map.info.width*(yi-expand)] > cost){
					cost = map.data[xi-expand+map.info.width*(yi-expand)];
				}
			
				geometry_msgs::Point pt;
				pt.x=x[i];	pt.y=y[i];	pt.z=0;
				count++;
			}
		}
		//-----------sawa--------------//
	}
	
	delete grid_b;
			
	return cost;
}

///////////////////////////////////////////////////////
/////-------Cordinate Convert function---------////////
///////////////////////////////////////////////////////

void pathLocal2Global(nav_msgs::Path& path, const tf::StampedTransform transform)
{
	int length=path.poses.size();
	nav_msgs::Odometry zero;
	float angle = tf::getYaw(transform.getRotation()) - 0;
	for(int i=0;i<length;i++){
		float tmp_x = path.poses[i].pose.position.x - zero.pose.pose.position.x;
		float tmp_y = path.poses[i].pose.position.y - zero.pose.pose.position.y;
		float conv_x = cos(angle)*tmp_x - sin(angle)*tmp_y;
		float conv_y = sin(angle)*tmp_x + cos(angle)*tmp_y;
		path.poses[i].pose.position.x = conv_x + transform.getOrigin().x();
		path.poses[i].pose.position.y = conv_y + transform.getOrigin().y();
	}
}

void localToGlobal(	geometry_msgs::Pose local_point,
					geometry_msgs::Pose& global,
					const tf::StampedTransform transform)
{
	float g_x = transform.getOrigin().x();
	float g_y = transform.getOrigin().y();
	float g_yaw = tf::getYaw(transform.getRotation());
	float l_x = local_point.position.x;
	float l_y = local_point.position.y;
	float l_yaw = tf::getYaw(local_point.orientation);
	float C = cos(g_yaw);
	float S = sin(g_yaw);
	
	float p_x = l_x*C - l_y*S + g_x;
	float p_y = l_x*S + l_y*C + g_y;
	
	geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,g_yaw+l_yaw);
	
	global.orientation = pose_quat;
	global.position.x = p_x;
	global.position.y = p_y;
}

float 
pathMake(const tf::StampedTransform trans, 
			float v, 
			float yawrate, 
			float time, 
			nav_msgs::Path& pa_, 
			const geometry_msgs::PoseStamped endp)
{
	float dt = 0.1;
	float xs2 = 0;
	float ys2 = 0;
	
	float theta = 0;
	float dis_cost;

	geometry_msgs::PoseStamped ps;
	geometry_msgs::PoseStamped ps_glo;
	geometry_msgs::Point pt;
	nav_msgs::Path pa;
	pa.header.frame_id="/map";
	pa.header.stamp=ros::Time::now();
	
	for(int j=0; j<time*10; j++){
		ps.pose.position.x=xs2 + dt*v*cos(theta);
		ps.pose.position.y=ys2 + dt*v*sin(theta);
		ps.pose.position.z=0.03;
		ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,theta);
		pa.poses.push_back(ps);	
		xs2 += dt*v*cos(theta);
		ys2 += dt*v*sin(theta);
		theta += yawrate*dt;
	}
	pathLocal2Global(pa, trans);
	//localToGlobal(ps.pose,ps_glo.pose,trans);
	pa_ = pa;
	float path_end_x = ps_glo.pose.position.x;
	float path_end_y = ps_glo.pose.position.y;
	dis_cost = distance(path_end_x, path_end_y, endp.pose.position.x, endp.pose.position.y);
	return dis_cost;
}
