//
//
//
#include "trajectory_generation/trajectory_generation.h"

#include <trajectory_generation/TrajectoryGeneration.h>
#include <Eigen/LU>

#include <ros/ros.h>
#include <iostream>
#define T 30

using namespace std;
const float delta_t = 0.1;
const float yawrate_max = 0.35;

	vector<float> v_profile;
	vector<float> s_profile;

void TrajectoryGeneration::fileInput(const string str, const float id)
{
	const char* file_name = str.c_str();
	ifstream ifs(file_name, ios::binary);
	ROS_INFO("Look Up Table setting....id:%f",id);
	if( !ifs ){
		//cout << "Error:Non file!!!!!---------Is this file's name correct?"<<endl;
		//cout << str <<endl;
		ROS_FATAL( "Error:Non file!!!!!---------Is this file's name correct?\n------->%s\n",str.c_str());
		exit(0);
	}else{
		float a;
		list<float> array;
		GoalPoint p;
		while(!ifs.eof()){  
			ifs.read( ( char * ) &a, sizeof( float ) );
			array.push_back(a);
		}
		float pre_k0 = 999;
		list<float>::iterator it = array.begin();
		while( it != array.end() ){
			if((*it) == 1234567){
				++it;
				p.path_end.pose.position.x = (*it);
				++it;
				p.path_end.pose.position.y = (*it);
				++it;
				p.path_end.pose.orientation.w = (*it);
				++it;
				p.output.curv.k0 = (*it);
				++it;
				p.output.curv.k1 = (*it);
				++it;
				p.output.curv.kf = (*it);
				++it;
				p.output.curv.sf = (*it);
				goal_p[id][p.output.curv.k0].push_back(p);
			}
			if(pre_k0!=p.output.curv.k0){
				k0_array.push_back(p.output.curv.k0);
			}
			pre_k0=p.output.curv.k0;
			++it;
		}	
		cout<<"size"<<goal_p[id].size()<<endl;;
	}
}


float TrajectoryGeneration::findIntersectK(float k0)
{
	list<float>::iterator it = k0_array.begin();
	float saiteki_k0 = 0;
	float min_cost = INFINITY;
	while( it != k0_array.end() ){
		float cost = fabs((*it) - k0);
		if(min_cost > cost){
			saiteki_k0 = (*it);
			min_cost = cost;
		}
		++it;
	}
	return saiteki_k0;
}

float TrajectoryGeneration::initSpeed2Id(float v0)
{
	float id_candidate = 0;
	int counter=0;
	map<float, map<float, list<GoalPoint> > >::iterator it = goal_p.begin();
	float min_cost = INFINITY;
	while( it != goal_p.end() )
	{
		float cost = fabs((*it).first - v0);
		if(min_cost > cost){
			min_cost = cost;
			id_candidate = (*it).first;
		}
		++it;
		counter++;
	}
	return id_candidate;
}

void TrajectoryGeneration::findOptimizedParam(Eigen::Vector3f goal,float k0,float v0,
												Eigen::Vector3f& params)
{
	float x = goal(0);
	float y = goal(1);
	float yaw = goal(2);
	float id = initSpeed2Id(v0);
	float map_key = findIntersectK(k0);
	
		//cout<<"key = "<<map_key<<endl;
	list<GoalPoint>::iterator it = goal_p[id][map_key].begin();
	GoalPoint saiteki;
	float min_cost = INFINITY;
	while( it != goal_p[id][map_key].end() ){
		float d_x = (*it).path_end.pose.position.x - x;
		float d_y = (*it).path_end.pose.position.y - y;
		float d_yaw = (*it).path_end.pose.orientation.w - yaw;
		float cost = sqrt(d_x*d_x + d_y*d_y + d_yaw*d_yaw);
		if(min_cost > cost){
			saiteki = (*it);
			min_cost = cost;
		}
		++it;
	}
	params(0) = saiteki.output.curv.k1;
	params(1) = saiteki.output.curv.kf;
	params(2) = saiteki.output.curv.sf;
}
/*----------------------------------------------------------------------------------*/

float
TrajectoryGeneration::planning(trajectory_generation::TrajectoryGeneration::Response& path_v,
								CntlParam& cntl_out, 
								const Eigen::Vector3f& goal, 
								const CntlParam& cntl0, 
								float dt, float tolerance, float max_count)
{
	cntl_out=cntl0;
	Eigen::Vector3f c(1,1,1), dp;
	int count=0;
		
	while(c.norm() > tolerance && count<max_count){
		
		cntl_out.velocity.tf=estimateTime(cntl_out.velocity, cntl_out.curv);
		cntl_out.curv.makeSpline();
		//cout<<"---------------makeVProfile---------------"<<endl;
		makeVProfile(dt, cntl_out.velocity);
		//cout<<"---------------makeVProfile after---------------"<<endl;
		/*for(float time=-t_delay; time<cntl_out.velocity.tf+dt; time+=dt){
			int ind = (time+t_delay)/dt;
			cout<<"Time:"<<time;
			cout<<"\t"<<v_profile[ind];
			cout<<"\t"<<s_profile[ind]<<endl;
		}*/
		
		
		if(cntl_out.velocity.tf>100)	break;
		Eigen::VectorXf x(5), x0(5), xf(5);
		x<<0,0,0,cntl_out.velocity.v0,cntl_out.curv.k0;
		
		path_v.path.poses.clear();
		path_v.v_array.vel.clear();
		for(float time=-t_delay; time<cntl_out.velocity.tf+dt; time+=dt){
			motionModel(xf, x, cntl_out, dt, time);
			x=xf;
			geometry_msgs::PoseStamped traj;
			traj.pose.position.x=x(0);
			traj.pose.position.y=x(1);		
			traj.pose.orientation.w=x(2);
			path_v.path.poses.push_back(traj);
			
			trajectory_generation::Velocity vel;
			vel.op_linear=x(3);
			vel.op_angular=x(4);
			path_v.v_array.vel.push_back(vel);
		}
		
		Eigen::Matrix3f J;
		makeJacob(J, dt, cntl_out, 0.005, 0.005, 0.1);
		//makeJacob(J, delta_t, cntl_out, 0.001, 0.001, 0.1);
		c<<goal(0)-x(0),goal(1)-x(1),goal(2)-x(2);
		dp=J.lu().solve(c);
		//cout<<"dp(0)="<< dp(0)<<"\tdp(1)="<< dp(1)<<"\tdp(2)="<< dp(2)<<endl;
		
		//cout<<"dp(0):"<<dp(0)<<"\tdp(1):"<<dp(1)<<"\tdp(2):"<<dp(2)<<endl;
		if ( isnan(dp(0)) || isnan(dp(1)) || isnan(dp(2)) || 
			isinf(dp(0)) || isinf(dp(1)) || isinf(dp(2)) 
			||(dp(0) > 5) || (dp(1) > 5) || (dp(2) > 20) ){
			cout<<"Diverge to infinity!!!"<<endl;
			return -1;
		}
		cntl_out.curv.k1+=dp(0);
		cntl_out.curv.kf+=dp(1);
		cntl_out.curv.sf+=dp(2);
		count++;
		
		
	}
	
	cout<<"Plan count is......"<<count<<endl;
	return c.norm();
}


geometry_msgs::PoseStamped
TrajectoryGeneration::saitekikaSinai(CntlParam& cntl_out, float dt)
{
	geometry_msgs::PoseStamped traj;
	cntl_out.velocity.tf=estimateTime(cntl_out.velocity, cntl_out.curv);
	cntl_out.curv.makeSpline();
	makeVProfile(dt, cntl_out.velocity);
	
	Eigen::VectorXf x(5), x0(5), xf(5);
	x<<0,0,0,cntl_out.velocity.v0,cntl_out.curv.k0;
	
	for(float time=-t_delay; time<cntl_out.velocity.tf+dt; time+=dt){
//		cout<<"time = "<<time<<endl;
		motionModel(xf, x, cntl_out, dt, time);
		x=xf;
		traj.pose.position.x=x(0);
		traj.pose.position.y=x(1);		
		traj.pose.orientation.w=x(2);
	}
	return traj;
}

nav_msgs::Path
TrajectoryGeneration::saitekikaSinaiPath(CntlParam& cntl_out, float dt)
{
	nav_msgs::Path path;
	geometry_msgs::PoseStamped traj;
	cntl_out.velocity.tf=estimateTime(cntl_out.velocity, cntl_out.curv);
	cntl_out.curv.makeSpline();
	makeVProfile(dt, cntl_out.velocity);
	
	Eigen::VectorXf x(5), x0(5), xf(5);
	x<<0,0,0,cntl_out.velocity.v0,cntl_out.curv.k0;
	
	for(float time=-t_delay; time<cntl_out.velocity.tf+dt; time+=dt){
//		cout<<"time = "<<time<<endl;
		motionModel(xf, x, cntl_out, dt, time);
		x=xf;
		traj.pose.position.x=x(0);
		traj.pose.position.y=x(1);		
//		traj.pose.orientation.w=x(2);
		path.poses.push_back(traj);
	}
	return path;
}

float
TrajectoryGeneration::estimateTime(const VelParam & upv, const CurvParam& upk){

	float t0=fabsf( (upv.vt-upv.v0)/upv.a0);
	float td=fabsf( (upv.vf-upv.vt)/upv.af);
	
	float s0=fabsf(upv.vt+upv.v0)*t0/2;
	float sd=fabsf(upv.vt+upv.vf)*td/2;

	float st=upk.sf-s0-sd;
	float tt=st/fabsf(upv.vt);
	float t=t0+td+tt;
	
	return t;
}

void
TrajectoryGeneration::motionModel(Eigen::VectorXf& x_next, const Eigen::VectorXf& x, const CntlParam& cntl, float dt, float time)
{
	Eigen::VectorXf out(5);
	out(0)=x(0)+x(3)*cosf(x(2))*dt;
	out(1)=x(1)+x(3)*sinf(x(2))*dt;
	out(2)=x(2)+x(3)*x(4)*dt;

	//out(3)=vProfile(time-t_delay, cntl.velocity);
	int ind = (time+t_delay)/dt;
	out(3)=v_profile[ind];
	//cout<<"Time:"<<time;
	//cout<<"\t"<<out(3)<<endl;
	
	//out(4)=curvProfile(time-t_delay, cntl.curv, cntl.velocity, dt);
	out(4)=curvProfile(time, cntl.curv, cntl.velocity, dt);
	responseToControlInputs(out, x, dt);
	
	x_next=out;
}

/*
float
TrajectoryGeneration::vProfile(float time, const VelParam& upv)
{
	if(time<0){
		return upv.v0;
	}
	float ta=fabs( (upv.vt-upv.v0)/upv.a0);
	float td=upv.tf - fabs( (upv.vf-upv.vt)/upv.af);
	
	float v=0;
	
	if(time<ta){
		v=upv.v0+time*upv.a0;
	}else if(ta <= time && time <= td){
		v=upv.vt;
	}else{
		v=upv.vt+(time-td)*upv.af;
		//cout<<"Time!!!"<<time<<"\tupv.tf"<<v<<endl;
	}
	
	if(time>upv.tf){
		v=upv.vf;
		
	}
	
	return v;
}*/


float
TrajectoryGeneration::curvProfile(float time, const CurvParam & upk, const VelParam& upv, float dt)
{
	//float s=0;
	float sf_2 = upk.sf/2;
	/*for(float t=0; t<time+dt; t+=dt){
		//s=s+vProfile(t, upv)*dt;
		s=s+ fabs(vProfile(t, upv) )*dt;
	}*/
	
	int ind = (time+t_delay)/dt;
	//float s = s_profile[int(time)*100];
	float s = s_profile[ind];
	
	float k;
	if(s <= sf_2){
		k = upk.p1[0]*s*s + upk.p1[1]*s + upk.p1[2];
	}else{
		float s2 = s - sf_2;
		k = upk.p2[0]*s2*s2 + upk.p2[1]*s2 + upk.p2[2];
	}
	
	return k;
}

void
TrajectoryGeneration::responseToControlInputs(Eigen::VectorXf& out, const Eigen::VectorXf& x, float dt)
{
	
	float k_cmd0 = x[4];
	float k_cmd1 = out[4];
	float dk_cmd = (k_cmd1 - k_cmd0)/dt;
	//--------Curvature rate Limit---------//
	dk_cmd = min(dk_cmd, dk_max);
	dk_cmd = max(dk_cmd, dk_min);
	speedControlLogic(out);
	k_cmd1 = k_cmd0 + dk_cmd*dt;
	
	//--------Curvature Limit---------//
	k_cmd1 = min(k_cmd1, k_max);
	k_cmd1 = max(k_cmd1, k_min);
	out[4] = k_cmd1;
	
	//--------Velocity Limit---------//
	float v_cmd0 = x[3];
	float v_cmd1 = out[3];
	float acc_cmd = (v_cmd1 - v_cmd0)/dt;
	acc_cmd = min(acc_cmd, acc_max);
	acc_cmd = max(acc_cmd, acc_min);
	out[3] = v_cmd0 + acc_cmd*dt;
}

void
TrajectoryGeneration::speedControlLogic(Eigen::VectorXf& out)
{
	//This is used in Urbanchallenge "Boss"
	/*
	float v_cmd_abs = fabs(out[3]);
	float v_cmd_max = max(v_scl,(out[4]-a_scl)/b_scl);
	float k_max_scl = min(k_max,(a_scl+b_scl*v_cmd_abs));
	if( fabs(out(4)) >= k_max_scl){
		v_cmd_abs = fabs( safety*v_cmd_max );
	}
	out[3] = v_cmd_abs * (out[3]/fabs(out[3]));
	*/
	float k_cmd_abs = fabs(out[4]);
	float yawrate = fabs(out[3] * out[4]);
	if(yawrate > yawrate_max){
		out[3] = (yawrate_max/k_cmd_abs) * (out[3]/fabs(out[3]));
	}
	
}

void
TrajectoryGeneration::makeJacob(Eigen::Matrix3f& J, float dt, const CntlParam& cntl, float dk1, float dkf, float dsf)
{
	Eigen::Vector3f x1, x2;
	Eigen::Matrix3f Jtmp;
	
	terminalState(x1, cntl.curv.k1-dk1, cntl.curv.kf, cntl.curv.sf, cntl, dt);
	terminalState(x2, cntl.curv.k1+dk1, cntl.curv.kf, cntl.curv.sf, cntl, dt);
	Eigen::Vector3f dc_dk1;
	//cout<<"x1:"<<x1.transpose()<<endl;
	//cout<<"x2:"<<x2.transpose()<<endl;
	dc_dk1<< (x2(0)-x1(0))/2.0/dk1, (x2(1)-x1(1))/2.0/dk1, (x2(2)-x1(2))/2.0/dk1;
	//cout<<"dc_dk1:"<<dc_dk1.transpose()<<endl;
		
	terminalState(x1, cntl.curv.k1, cntl.curv.kf-dkf, cntl.curv.sf, cntl, dt);
	terminalState(x2, cntl.curv.k1, cntl.curv.kf+dkf, cntl.curv.sf, cntl, dt);
	Eigen::Vector3f dc_dkf;
	dc_dkf<< (x2(0)-x1(0))/2/dkf, (x2(1)-x1(1))/2/dkf, (x2(2)-x1(2))/2/dkf;	
	
	terminalState(x1, cntl.curv.k1, cntl.curv.kf, cntl.curv.sf-dsf, cntl, dt);
	terminalState(x2, cntl.curv.k1, cntl.curv.kf, cntl.curv.sf+dsf, cntl, dt);
	Eigen::Vector3f dc_dsf;
	dc_dsf<< (x2(0)-x1(0))/2/dsf, (x2(1)-x1(1))/2/dsf, (x2(2)-x1(2))/2/dsf;	
	
	J<<	dc_dk1(0), dc_dkf(0), dc_dsf(0),
			dc_dk1(1), dc_dkf(1), dc_dsf(1),
			dc_dk1(2), dc_dkf(2), dc_dsf(2);
	//cout<<"Jtmp:\n"<<Jtmp<<endl;
	//J=Jtmp;
}

void
TrajectoryGeneration::terminalState(Eigen::Vector3f& x, float k1, float kf, float sf, const CntlParam& cntl, float dt)
{
	CntlParam ct(cntl.curv, cntl.velocity);
	ct.curv.k1=k1;
	ct.curv.kf=kf;
	ct.curv.sf=sf;	
	ct.velocity.tf=estimateTime(ct.velocity, ct.curv);
	ct.curv.makeSpline();
	makeVProfile(dt, ct.velocity);
	
	Eigen::VectorXf x0(5), xf(5);
	x0<<0,0,0,ct.velocity.v0,ct.curv.k0;
	
	//cout<<"dt:"<<dt<<"motion_step:"<<(ct.velocity.tf+dt)/dt<<endl;
	for(float time=-t_delay; time<ct.velocity.tf+dt; time+=dt){
		motionModel(xf, x0, ct, dt, time);
		x0=xf;
		//cout<<x.transpose()<<endl;
	}
	
	x(0)=x0(0);	x(1)=x0(1);	x(2)=x0(2);
}


void TrajectoryGeneration::makeVProfile(float dt, const VelParam& upv)
{
	//cout<<"---------------------------------into!"<<endl;
	if(v_profile.size()>0)	v_profile.clear();
	if(s_profile.size()>0)	s_profile.clear();
	//cout<<v_profile.size()<<endl;
	float s=0;
	for(float time=-t_delay; time<upv.tf+dt; time+=dt){
		
		float ta=fabsf( (upv.vt-upv.v0)/upv.a0);
		float td=upv.tf - fabsf( (upv.vf-upv.vt)/upv.af);
		
		float v=0;
		if(time<0){
			v=upv.v0;
		}else if(time<ta){
			v=upv.v0+time*upv.a0;
		}else if(ta <= time && time <= td){
			v=upv.vt;
		}else{
			v=upv.vt+(time-td)*upv.af;
			//cout<<"Time!!!"<<time<<"\tupv.tf"<<v<<endl;
		}
		
		if(time>upv.tf){
			v=upv.vf;
			
		}
		
		//v_profile[time] = v;
		//v_profile.insert(map<int, float>::value_type( int(time*100), v ));
		v_profile.push_back(v);
		
	//cout<<"---------------------------------v_profile OK!"<<endl;
		s += fabs(v)*dt;
	//cout<<"---------------------------------s_profile[time]!"<<endl;
		//s_profile[time] = s;
		//s_profile.insert(map<int, float>::value_type( int(time*100), s ));
		s_profile.push_back(s);
	//cout<<"v_profile[time]!:"<<time<<"\tv:"<<v<<"\ts:"<<s<<endl;
	}
	//cout<<"---------------------------------out!"<<endl;
}




void
CurvParam::makeSpline()
{
	x[0]=0;	x[1]=sf/2.0;	x[2]=sf;
	y[0]=k0;	y[1]=k1;	y[2]=kf;
	
	/*Eigen::MatrixXf S(6,6);
	Eigen::VectorXf c(6);
	Eigen::VectorXf ans;
	
	S<<	x[0]*x[0],x[0],1, 0,0,0,
		x[1]*x[1],x[1],1, 0,0,0,
		0,0,0, x[1]*x[1],x[1],1,
		0,0,0, x[2]*x[2],x[2],1,
		2.0*x[1],x[1],0, -2.0*x[1],x[1],0,
		1,0,0, 0,0,0;
	
	c<<y[0],y[1],y[1],y[2],0,0;*/
	Eigen::MatrixXf S(3,3);
	Eigen::VectorXf c(3);
	Eigen::VectorXf ans;
	
	S<<	x[1]*x[1],x[1],0,
		x[1]*x[1],0,x[1],
		x[2],1,-1;
	
	c<<y[1]-y[0],y[2]-y[1],0;
//	S.lu().solve(c, &ans);
	ans=S.lu().solve(c);
	p1[0]=ans(0);	p1[1]=ans(1);	p1[2]=y[0];
	p2[0]=p1[0];	p2[1]=ans(2);	p2[2]=y[1];	
}

