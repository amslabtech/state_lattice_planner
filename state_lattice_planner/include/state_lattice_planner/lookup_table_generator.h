#ifndef __LOOKUP_TABLE_GENERATOR_H
#define __LOOKUP_TABLE_GENERATOR_H

#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include <Eigen/Dense>

#include "trajectory_generator/motion_model_diff_drive.h"
#include "trajectory_generator/trajectory_generator_diff_drive.h"

class LookupTableGenerator
{
public:
    LookupTableGenerator(void);

    std::string process(void);
    void save(std::string&);

private:
    float MIN_X;
    float MAX_X;
    float DELTA_X;
    float MAX_Y;
    float DELTA_Y;
    float MAX_YAW;
    float DELTA_YAW;
    std::string LOOKUP_TABLE_FILE_NAME;
    float TARGET_VELOCITY;
    float MIN_V;
    float MAX_V;
    float DELTA_V;
    float MAX_KAPPA;
    float DELTA_KAPPA;
    float MAX_ACCELERATION;
    float MAX_CURVATURE;
    float MAX_D_CURVATURE;
    float MAX_YAWRATE;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

};

#endif// __LOOKUP_TABLE_GENERATOR_H
