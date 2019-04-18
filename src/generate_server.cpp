#include "ros/ros.h"
#include <trajectory_generation/TrajectoryGeneration.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int32.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <string>
#include <list>
#include <map>
#include <time.h>

//---------------From library---------------//
#include "trajectory_generation/Visualize_lib.h"
#include "trajectory_generation/trajectory_generation.h"
#include "trajectory_generation/boundary_state.h"
#include "trajectory_generation/LocalPlanning_lib.h"

#define TRACE 1

const float Vmax = 0.8;	// infantの設定最高速度
const float Vmax_ = 1.0/Vmax;	// infantの設定最高速度
const float Lmin = 2.0;
//const float Lmax = 6.0;
const float Lmax = 5.0;
// const float MaxAngle = 1.2;
const float MaxAngle = 1.2;

//const string header_frame("/map");
//const string header_frame2("/matching_base_link");
const string header_frame("/localmap");
const string header_frame2("/velodyne_odom");
//const string header_frame2("/velodyne");
using namespace std;

//-----------Pathの構造体-----------//
//パス一本一本に対して情報を付加します//
typedef struct {
	float cost;		//優先順位をつけるためのコスト
	nav_msgs::Path path;
	trajectory_generation::VelocityArray v_a;
	u_int id;
	geometry_msgs::PoseStamped goal;
} PathState;

TrajectoryGeneration trajectory(16.0, -16.0,		//curvature
								16.0, -16.0,			//rate of curvature
								0.4, -0.4,				//acceleration
								0.00,					//command delay
								10.1681, -0.0049, 0.9,	//Speed Control logic
								2.0,					//Maxcurvature for speed
								1.0);					//Safety factor


//ShapeParameter p_ss = {1000, 31, 3, 7.0, -1.6, 1.6, -0.7, 0.7};
//ShapeParameter p_ss = {1000, 9, 3, 7.0, -0.8, 0.8, -0.7, 0.7};	//20151101
//ShapeParameter p_ss = {1000, 9, 5, 7.0, -1.2, 1.2, -0.9, 0.9};	//20151105
ShapeParameter p_ss = {1000, 31, 5, 7.0, -1.6, 1.6, -0.9, 0.9};	//20181024
//	{n_s:固定値，n_p:目的地数，n_h:目的地あたりの本数，d:pathの長さ，alpha_min:角度，alpha_max:角度，psi_min(max)目的地での角度の開き}


class MY_LESS_DEFINITION{
public:
	bool operator() (const PathState& left, const PathState& right)
	{
		return left.cost < right.cost;
	}
};

//////////////////////////////////////////////////
//-------mutexロック スコープの範囲だけ有効-------//
//////////////////////////////////////////////////
ros::Publisher _path_pub;
boost::mutex path_pub_mutex_;
ros::Publisher _pose_pub;
boost::mutex pose_pub_mutex_;
ros::Publisher _array_pub;
boost::mutex array_pub_mutex_;
ros::Publisher _num_pub;
boost::mutex num_pub_mutex_;
ros::Publisher _target_pub;
boost::mutex target_pub_mutex_;

/////////////////////////////////////////////////////////
//-------------------CallBack!!!!!---------------------//
/////////////////////////////////////////////////////////
bool callback_flag=false;
nav_msgs::OccupancyGrid grd_;
boost::mutex map_mutex_;
void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg){
	boost::mutex::scoped_lock(map_mutex_);	
	grd_=*msg;
	grd_.info.origin.position.z = 0;////////
	for(int i=0;i<grd_.info.width*grd_.info.height;i++){
		if(grd_.data[i]>0) grd_.data[i] = 100;
	}
	callback_flag=true;
}

double vel_now = 0.0;
boost::mutex odom_mutex_;
void odomCallback(const nav_msgs::OdometryConstPtr& msg){
	boost::mutex::scoped_lock(odom_mutex_);
	vel_now = msg->twist.twist.linear.x;
}

int wp_mode=0;
void WpModeCallback(const std_msgs::Int32ConstPtr& msg)
{
	wp_mode=msg->data;
}


void pathLocalGlobal(nav_msgs::Path& path, geometry_msgs::PoseStamped loc)
{
	int length=path.poses.size();
	nav_msgs::Odometry zero;
	float angle = tf::getYaw(loc.pose.orientation) - 0;
	for(int i=0;i<length;i++){
		float tmp_x = path.poses[i].pose.position.x - zero.pose.pose.position.x;
		float tmp_y = path.poses[i].pose.position.y - zero.pose.pose.position.y;
		float conv_x = cos(angle)*tmp_x - sin(angle)*tmp_y;
		float conv_y = sin(angle)*tmp_x + cos(angle)*tmp_y;
		path.poses[i].pose.position.x = conv_x + loc.pose.position.x;
		path.poses[i].pose.position.y = conv_y + loc.pose.position.y;
	}
}

void showPath(nav_msgs::Path path)
{
	std_msgs::ColorRGBA rgba_in;
	rgba_in.r = 1.0;
	rgba_in.g = 0;
	rgba_in.b = 0;
	rgba_in.a = 0.8;
	Visualization* marker_line = new Visualization(rgba_in, ADD,0.1, "OnlyPath",header_frame);
	ros::Publisher pub;
	{
		boost::mutex::scoped_lock(path_pub_mutex_);	
		pub = _path_pub;
	}
	
	visualization_msgs::Marker line1;
	marker_line->convertPath2MarkerLine(path, line1, 1);
		
	// pub.publish(line1);
	delete marker_line;
}

string IntToString(int number)
{
	stringstream ss;
	ss << number;
	return ss.str();
}

void showPathArray(list<PathState> path_array, int num)
{
	std_msgs::ColorRGBA rgba_in;
	rgba_in.r = 0.0;
	rgba_in.g = 0.8;
	rgba_in.b = 0.9;
	//rgba_in.a = 0.8;
	rgba_in.a = 1.0;
	Visualization* marker_line = new Visualization(rgba_in, ADD,0.02, "PathSet",header_frame);
	Visualization* marker_txt = new Visualization(rgba_in, ADD,0.02, "txtSet",header_frame);
	ros::Publisher pub1,pub2;
	{
		boost::mutex::scoped_lock(array_pub_mutex_);	
		pub1 = _array_pub;
	}
	{
		boost::mutex::scoped_lock(num_pub_mutex_);	
		pub2 = _num_pub;
	}
	int i = 0;
	//------------------line array---------------------------//
	visualization_msgs::MarkerArray lines, txts;
	list<PathState>::iterator it = path_array.begin();
	while( it != path_array.end() ){
		visualization_msgs::Marker line1, txt1;
		nav_msgs::Path path;
		//if((*it).cost<100)	
		path = (*it).path;
		int id = (*it).id;
		marker_line->convertPath2MarkerLine(path, line1, i);
		marker_txt->txtMarker(IntToString(id), (*it).goal, txt1, i);
		lines.markers.push_back(line1);
		txts.markers.push_back(txt1);
		i++;
		++it;
	}//un-show useless path
	for(; i<num; i++){
		visualization_msgs::Marker line2;
		nav_msgs::Path path_zero;
		marker_line->convertPath2MarkerLine(path_zero, line2, i);
		lines.markers.push_back(line2);
	}
	
	//------------------txt array---------------------------//
	
	
	pub1.publish(lines);
	
	//pub2.publish(txts);
	delete marker_line;
}

void showPoses(geometry_msgs::PoseArray poses)
{
	ros::Publisher pub;
	{
		boost::mutex::scoped_lock(poses_pub_mutex_);
		pub = _pose_pub;
	}
	//poses.header.frame_id = header_frame;
	poses.header.frame_id = header_frame2;
	poses.header.stamp=ros::Time::now();
	
	pub.publish(poses);
}
void showPose(geometry_msgs::PoseStamped target)
{
	ros::Publisher pub;
	{
		boost::mutex::scoped_lock(target_pub_mutex_);
		pub = _target_pub;
	}
	target.header.frame_id = header_frame;
	target.header.stamp=ros::Time::now();
	
	//pub.publish(target);
}



float trajectoryLengthControl2(float theta)
{
	float L = -(Lmax-Lmin)*0.637*fabs(theta) + Lmax;	// 0.637 : 2/PI
	L *= vel_now*Vmax_;
	if(L>Lmax){
		L = Lmax;
	}else if(L<Lmin){
		L = Lmin;
	}

	return L;
}

///////////////////////////////////////////////////////
/////--------------Target maker----------------////////
///////////////////////////////////////////////////////
void targetMaker(trajectory_generation::TrajectoryGeneration::Request req, 
					geometry_msgs::PoseStamped& x_f)
{
	//p_ss.d = req.r;
	float theta = req.goal.pose.position.x;
	p_ss.d = trajectoryLengthControl2(theta);
	x_f.pose.position.x = p_ss.d * cos(theta);
	x_f.pose.position.y = p_ss.d * sin(theta);
	x_f.pose.position.z = 0;
	x_f.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,theta);

	//p_ss.alph_min = theta - 0.35;	// 0.52 : 30[deg], 0.35 : 20[deg]
	//p_ss.alph_max = theta + 0.35;	// 0.52 : 30[deg]
	p_ss.alph_min = theta - 0.786;	// 0.52 : 30[deg], 0.35 : 20[deg]
	p_ss.alph_max = theta + 0.786;	// 0.52 : 30[deg]
	if(p_ss.alph_min < -MaxAngle){
		p_ss.alph_min = -MaxAngle;
		p_ss.alph_max = -MaxAngle+0.7;
	}
	if(p_ss.alph_max > MaxAngle){
		p_ss.alph_max = MaxAngle;
		p_ss.alph_min = MaxAngle-0.7;
	}
}

void knmConvertor(trajectory_generation::TrajectoryGeneration::Request req,
					geometry_msgs::PoseStamped& x_f)
{
	//p_ss.d = req.r;
	//float theta = req.goal.pose.position.x;
	float theta = tf::getYaw(req.goal.pose.orientation);
	p_ss.d = trajectoryLengthControl2(theta);
	x_f.pose.position.x = p_ss.d * cos(theta);
	x_f.pose.position.y = p_ss.d * sin(theta);
	x_f.pose.position.z = 0;
	x_f.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,theta);

	// p_ss.alph_min = theta - 0.35;	// 0.52 : 30[deg], 0.35 : 20[deg]
	// p_ss.alph_max = theta + 0.35;	// 0.52 : 30[deg]
	p_ss.alph_min = theta - 0.69;	// 0.52 : 30[deg], 0.35 : 20[deg]
	p_ss.alph_max = theta + 0.69;	// 0.52 : 30[deg]
	if(p_ss.alph_min < -MaxAngle){
		p_ss.alph_min = -MaxAngle;
		// p_ss.alph_max = -MaxAngle+0.7;
	}
	if(p_ss.alph_max > MaxAngle){
		p_ss.alph_max = MaxAngle;
		// p_ss.alph_min = MaxAngle-0.7;
	}
}

///////////////////////////////////////////////////////
/////----------BoundaryStates function---------////////
///////////////////////////////////////////////////////
float setBoundaryStates(trajectory_generation::TrajectoryGeneration::Request req,
						geometry_msgs::PoseArray& boundary_state)
{
	float return_n;
	geometry_msgs::PoseStamped x_i;
	x_i.pose.position.x=0;
	x_i.pose.position.y=0;
	x_i.pose.position.z=0;
	x_i.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0.0);
	return_n = (p_ss.n_p * p_ss.n_h);
	//p_ss.d = req.r;
	//cout<<"req.r:"<<req.r<<endl;
	//p_ss.d = vel_now*5+2.0;
	if (wp_mode==TRACE) p_ss.d=0.5;//trace modeとそろえること
//--------Teledrive Mode !!
	/*if(req.teledrive){
		geometry_msgs::PoseStamped x_f;
		targetMaker(req, x_f);
		generateGloballyGuidedBoundaryStates(p_ss,x_i, x_f, boundary_state);
//--------Autonomy Mode !!
	}else{*/
		if(!req.fin){
			//Forward trajectory
			if((req.params.vt+req.params.vf) > 0){
				return_n = (p_ss.n_p * p_ss.n_h);
				geometry_msgs::PoseStamped x_f;
				knmConvertor(req, x_f);
				cout<<"req.goal: "<<req.goal<<endl;
				cout<<"x_f: "<<x_f<<endl;
				generateGloballyGuidedBoundaryStates(p_ss,x_i, x_f, boundary_state);
				//generateGloballyGuidedBoundaryStates(p_ss,x_i, req.goal, boundary_state);
			//Back trajectory
			}else{
				p_ss.d = -1.2;
				p_ss.psi_min=-0.2;
				p_ss.psi_max=0.2;
				p_ss.n_p = 13;
				generateUniformBoundaryStates(p_ss,x_i,boundary_state);
			}
		/*}else{
			//cout<<"---------------------goal!!--------------------"<<req.goal<<endl;
			return_n = 10;
			generateFinalBoundaryStates(p_ss, return_n, x_i, req.goal, boundary_state);
		}*/
	}
	showPoses(boundary_state);
	return return_n;
}

///////////////////////////////////////////////////////
/////-----------Evaluation function------------////////
///////////////////////////////////////////////////////
int pickUpTrajectory(list<PathState> path_array, 
						PathState& pick_up_path, 
						geometry_msgs::PoseStamped goal)
{
	vector<PathState> path_array_tmp;
	for(list<PathState>::iterator it = path_array.begin();it != path_array.end(); ++it){
		PathState pst;
		pst = (*it);
		float dx = goal.pose.position.x - (*it).goal.pose.position.x;
		float dy = goal.pose.position.y - (*it).goal.pose.position.y;
		pst.cost = sqrt(dx*dx + dy*dy);
		path_array_tmp.push_back(pst);
	}
	
	sort(path_array_tmp.begin(), path_array_tmp.end(), MY_LESS_DEFINITION());
	showPose(goal);
	float min_cost = INFINITY;
	int iii=0;
	std::cout<<"num candidate of path: "<< path_array_tmp.size()<< std::endl;
	for(vector<PathState>::iterator it = path_array_tmp.begin();it != path_array_tmp.end(); ++it){
		float dyaw = fabs(tf::getYaw(goal.pose.orientation)-tf::getYaw((*it).goal.pose.orientation));
		if(min_cost > dyaw){
			min_cost = dyaw;
			pick_up_path = (*it); 
		}
		iii++;
		if(!(iii<3))	break;
	}
	
	if(path_array.size()<1)	return -1;
	
	return 0;
}

void generatePathArray(trajectory_generation::TrajectoryGeneration::Request req,
						geometry_msgs::PoseArray boundary_state, 
						list<PathState>& path_array)
{
	
	nav_msgs::OccupancyGrid gmap;
	{
		boost::mutex::scoped_lock(map_mutex_);
		gmap=grd_;
	}
	if (callback_flag){
		for(u_int i=0; i<boundary_state.poses.size(); i++){
			PathState path_state;
			float x = boundary_state.poses[i].position.x;
			float y = boundary_state.poses[i].position.y;
			float yaw = tf::getYaw(boundary_state.poses[i].orientation);
			Eigen::Vector3f goal(x, y, yaw);
			
			float speed_ave = (req.params.v0 + req.params.vt + req.params.vf)/3;
			
			Eigen::Vector3f params;
			trajectory.findOptimizedParam(goal, req.params.k0, speed_ave, params);
			
			CntlParam out;
			CntlParam in(CurvParam(req.params.k0, params(0), params(1), params(2)),
						VelParam(req.params.v0,req.params.a0,req.params.vt,req.params.af,req.params.vf));
			
			trajectory_generation::TrajectoryGeneration::Response path_v;
			float tol = trajectory.planning(path_v, out, goal, in, 0.1, 0.1, 20);
			
			path_state.path = path_v.path;
			pathLocalGlobal(path_state.path, req.start);
			path_state.cost = pathCheck(path_state.path, gmap);
			path_state.v_a = path_v.v_array;
			path_state.id = i;
			path_state.goal.pose = boundary_state.poses[i];//input goal
			if((tol!=-1) && (path_state.cost<100))	path_array.push_back(path_state);
			//cout<<"Cost:"<<path_state.cost<<endl;
			//path_array.push_back(path_state);
		}
	}
}


bool server(trajectory_generation::TrajectoryGeneration::Request  &req,
         trajectory_generation::TrajectoryGeneration::Response &res )
{
	clock_t start,end;
	start = clock();
	
//--------Teledrive Mode !!
	if(req.teledrive){
		geometry_msgs::PoseStamped x_f;
		targetMaker(req, x_f);
		showPose(x_f);
		req.goal = x_f;
	}
    // cout<<"bbb"<<endl;
	geometry_msgs::PoseArray boundary_state;
	float state_n = setBoundaryStates(req, boundary_state);
    // cout<<"ccc"<<endl;
    ///////////////cout<<state_n<<endl;
	
	list<PathState> path_array;
	generatePathArray(req, boundary_state, path_array);
	
	PathState pick_up_path;
	showPose(req.goal);
	req.goal.header.frame_id = header_frame;
	cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << req.goal << endl;
	res.tolerance = pickUpTrajectory(path_array, pick_up_path, req.goal);
	
	res.path.poses.clear();
	res.v_array.vel.clear();
	res.path = pick_up_path.path;
	res.v_array = pick_up_path.v_a;
	// res.tolerance=tol;
	res.path.header.frame_id=header_frame;
	res.path.header.stamp=req.header.stamp;
	
	showPathArray(path_array, state_n);
	
    //showPath(res.path);
	
	end = clock();
	ROS_INFO("Responce Time: %f", (double)(end-start)/CLOCKS_PER_SEC);
	return true;
}


int main(int argc, char **argv)
{
	const string package_path_str("/home/amsl/ros_catkin_ws/src/state_lattice_planner/");
	trajectory.fileInput(package_path_str+"/look_up_table/infant/v-03.bin",-0.3);
	trajectory.fileInput(package_path_str+"/look_up_table/infant/v-02.bin",-0.2);
	trajectory.fileInput(package_path_str+"/look_up_table/infant/v-01.bin",-0.1);
	trajectory.fileInput(package_path_str+"/look_up_table/infant/v01.bin",0.1);
	trajectory.fileInput(package_path_str+"/look_up_table/infant/v02.bin",0.2);
	trajectory.fileInput(package_path_str+"/look_up_table/infant/v03.bin",0.3);
	trajectory.fileInput(package_path_str+"/look_up_table/infant/v04.bin",0.4);
	trajectory.fileInput(package_path_str+"/look_up_table/infant/v05.bin",0.5);
	trajectory.fileInput(package_path_str+"/look_up_table/infant/v06.bin",0.6);
	trajectory.fileInput(package_path_str+"/look_up_table/infant/v07.bin",0.7);
	trajectory.fileInput(package_path_str+"/look_up_table/infant/v08.bin",0.8);
	trajectory.fileInput(package_path_str+"/look_up_table/infant/v09.bin",0.9);
	ROS_INFO("Look Up Table complete!!!");
	ros::init(argc, argv, "generate_server");
	ros::NodeHandle n;
	
	//ros::Publisher pub1 = n.advertise<visualization_msgs::Marker>("plan/localpath_vis", 100);
	//ros::Publisher pub3 = n.advertise<visualization_msgs::MarkerArray>("plan/path_recomend", 100);
	ros::Publisher pub3 = n.advertise<visualization_msgs::MarkerArray>("plan/path_recomend_shogo", 100);
	ros::Publisher pub2 = n.advertise<geometry_msgs::PoseArray>("plan/poses", 100);
	ros::ServiceServer service = n.advertiseService("/plan/local_path", server);
	ros::Subscriber sub1 = n.subscribe("/local_map", 5, mapCallback);
	//ros::Subscriber sub1 = n.subscribe("/local_map/expand/gc", 5, mapCallback);
	ros::Subscriber sub_odom = n.subscribe("/tinypower/odom", 5, odomCallback);
	ros::Subscriber mode_sub = n.subscribe("/wp_mode", 1, WpModeCallback);
	// ros::Subscriber sub1 = n.subscribe("/local_map/global_coord/expanded", 1, mapCallback);
	// ros::Subscriber sub1 = n.subscribe("/local_map/global_coord", 1, mapCallback);
	// ros::Subscriber sub1 = n.subscribe("/map/local_map_with_gc", 100, mapCallback);//20141013 written by masada
	
	{
		boost::mutex::scoped_lock(pose_pub_mutex_);	
		_pose_pub = pub2;
	}
	{
		boost::mutex::scoped_lock(array_pub_mutex_);	
		_array_pub = pub3;
	}
	ROS_INFO("generate_server start.");
	
	ros::spin();

	return 0;
}
