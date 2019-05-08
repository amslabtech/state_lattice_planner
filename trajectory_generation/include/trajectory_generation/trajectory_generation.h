//
//
//
#ifndef TRAJECTORY_GENERATION_H
#define TRAJECTORY_GENERATION_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <math.h>
#include <list>
#include <fstream>
#include <trajectory_generation/TrajectoryGeneration.h>
#include <iostream>
/*
#define	K_MAX	0.5
#define	K_MIN	-0.5
#define	V_MAX	0.2
#define	V_MIN	0.0
#define AC_MAX	0.4
#define AC_MIN	-0.4
*/
using namespace std;



class CurvParam{
private:
	CurvParam();
	
public:
	CurvParam(float _k0, float _k1, float _kf, float _sf)
					:k0(_k0), k1(_k1), kf(_kf), sf(_sf){}
	
	CurvParam(const CurvParam& param)
	:k0(param.k0), k1(param.k1), kf(param.kf), sf(param.sf)
	{
		for(int i=0; i<3; i++){
			x[i]=param.x[i];
			y[i]=param.y[i];
			p1[i]=param.p1[i];
			p2[i]=param.p2[i];
			
		}
	}
	
	void makeSpline();
		
	float k0, k1, kf, sf;
	float x[3], y[3], p1[3], p2[3];
};

class VelParam{
private:
	VelParam();
	
public:
	VelParam(float _v0, float _a0, float _vt, float _af, float _vf, float _tf=0)
					:v0(_v0), a0(_a0), vt(_vt), af(_af), vf(_vf), tf(_tf){}
	
	VelParam(const VelParam& param)
					:v0(param.v0), a0(param.a0), vt(param.vt), af(param.af), vf(param.vf), tf(param.tf){}
	
	float v0, a0, vt, af, vf, tf;
	
};


class CntlParam{
private:
	
public:
	CntlParam():curv(0,0,0,0), velocity(0,0,0,0,0,0){};
		
	CntlParam(const CurvParam& _curv, const VelParam& _vel)
					:curv(_curv), velocity(_vel){}
	
	CntlParam(const CntlParam& param)
					:curv(param.curv), velocity(param.velocity){}
	CntlParam& operator=(const CntlParam& param){
		
		curv.k0=param.curv.k0;
		curv.k1=param.curv.k1;
		curv.kf=param.curv.kf;
		curv.sf=param.curv.sf;
		for(int i=0; i<3; i++){
			curv.x[i]=param.curv.x[i];
			curv.y[i]=param.curv.y[i];
			curv.p1[i]=param.curv.p1[i];
			curv.p2[i]=param.curv.p2[i];
			
		}
		velocity.v0=param.velocity.v0;
		velocity.a0=param.velocity.a0;
		velocity.vt=param.velocity.vt;
		velocity.af=param.velocity.af;
		velocity.vf=param.velocity.vf;
		velocity.tf=param.velocity.tf;
		
		return *this;
	}
	
	CurvParam curv;
	VelParam velocity;
};


typedef struct {
	geometry_msgs::PoseStamped path_end;
	CntlParam output;
} GoalPoint;


	//vector<float> v_profile;
class TrajectoryGeneration{
private:

	float k_max;
	float k_min;
	float dk_max;
	float dk_min;
	float acc_max;
	float acc_min;
	float t_delay;
	float a_scl;
	float b_scl;
	float v_scl;
	float kv_max;
	float safety;
	
	map<float, map<float, list<GoalPoint> > > goal_p;
	list<float> k0_array;
	//map<float,float> v_profile;
	//map<float,float> s_profile;

	void makeJacob(Eigen::Matrix3f& J, float dt, const CntlParam& cntl, float dk1, float dkf, float dsf);
	void terminalState(Eigen::Vector3f& x, float k1, float kf, float sf, const CntlParam& cntl, float dt);
	TrajectoryGeneration(const TrajectoryGeneration&);
public:
	TrajectoryGeneration(float _k_max,
						float _k_min,
						float _dk_max,
						float _dk_min,
						float _acc_max,
						float _acc_min,
						float _t_delay,
						float _a_scl,
						float _b_scl,
						float _v_scl,
						float _kv_max,
						float _safety)
	{
		k_max = _k_max;
		k_min = _k_min;
		dk_max = _dk_max;
		dk_min = _dk_min;
		acc_max = _acc_max;
		acc_min = _acc_min;
		t_delay = _t_delay;
		a_scl = _a_scl;
		b_scl = _b_scl;
		v_scl = _v_scl;
		kv_max = _kv_max;
		safety = _safety;
	}
	~TrajectoryGeneration(){}
	
	
	//vector<float> v_profile;
	//vector<float> s_profile;
	
	float planning(trajectory_generation::TrajectoryGeneration::Response& path_v,
					CntlParam& cntl_out, 
					const Eigen::Vector3f& goal, 
					const CntlParam& cntl0, 
					float dt=0.1,float tolerance=0.05, float max_count=20);
	
	geometry_msgs::PoseStamped saitekikaSinai(CntlParam& cntl_out, float dt);
	nav_msgs::Path saitekikaSinaiPath(CntlParam& cntl_out, float dt);
	void motionModel(Eigen::VectorXf& x_next, const Eigen::VectorXf& x, const CntlParam& cntl, float dt, float time);
	
	/*------------look up table---------------*/
	void fileInput(const string str, const float id);
	void findOptimizedParam(Eigen::Vector3f goal,float k0,float v0, Eigen::Vector3f& params);
	float initSpeed2Id(float v0);
	float findIntersectK(float k0);
	/*-----------------------------------------*/

	void responseToControlInputs(Eigen::VectorXf& out, const Eigen::VectorXf& x, float dt);
	void speedControlLogic(Eigen::VectorXf& out);
	float vProfile(float time, const VelParam& upv);
	float curvProfile(float time, const CurvParam & upk, const VelParam& upv, float dt);
	float estimateTime(const VelParam& upv, const CurvParam & upk);
	void makeVProfile(float dt,const VelParam& upv);
	
	//void makeSpline(const CurvParam& upk);
};

#endif  // TRAJECTORY_GENERATION_H
