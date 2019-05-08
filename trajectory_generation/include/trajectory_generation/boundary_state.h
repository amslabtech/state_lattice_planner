//
//
//

#ifndef BOUNDARY_STATE_H
#define BOUNDARY_STATE_H

#include "ros/ros.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <string>
#include <list>
#include <iostream>
#include <math.h>
#include <tf/transform_listener.h>

using namespace std;

typedef struct {
	int n_s;
	int n_p;
	int n_h;
	float d;
	float alph_min;
	float alph_max;
	float psi_min;
	float psi_max;
} ShapeParameter;

typedef struct {
	nav_msgs::Path l_center;
	float l_heading;
	float l_width;
	float v_width;
	float d;
	int n_p;
	int n_l;
}LGShapeParameter;

void generateUniformBoundaryStates(ShapeParameter p_ss, geometry_msgs::PoseStamped x_i, 
									geometry_msgs::PoseArray& boundary_state);
									

void generateFinalBoundaryStates(ShapeParameter p_ss, int num,
									geometry_msgs::PoseStamped x_i, 
									geometry_msgs::PoseStamped x_d, 
									geometry_msgs::PoseArray& boundary_state);
											
float navFunc(float x, float y, geometry_msgs::PoseStamped x_d);

float listSum(list<float> a);

float findIntersectionCnav(ShapeParameter p_ss, list<float> cnav, float val);
void generateGloballyGuidedBoundaryStates(ShapeParameter p_ss, 
											geometry_msgs::PoseStamped x_i, 
											geometry_msgs::PoseStamped x_d, 
											geometry_msgs::PoseArray& boundary_state);
											
void computeStateAtDistanceAlongLane(nav_msgs::Path l_center, float d,
										geometry_msgs::PoseStamped& x_center);

void generateLaneGuidedBoundaryStates(LGShapeParameter p_ss, geometry_msgs::PoseStamped x_i, 
									geometry_msgs::PoseArray& boundary_state);
void generateLaneGuidedBoundaryStates2(LGShapeParameter p_ss, geometry_msgs::PoseStamped x_i, geometry_msgs::PoseStamped x_d, 
									geometry_msgs::PoseArray& boundary_state);

#endif // BOUNDARY_STATE_H

