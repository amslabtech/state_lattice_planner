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
    double MIN_X;
    double MAX_X;
    double DELTA_X;
    double MAX_Y;
    double DELTA_Y;
    double MAX_YAW;
    double DELTA_YAW;
    std::string LOOKUP_TABLE_FILE_NAME;
    double MAX_ACCELERATION;
    double TARGET_VELOCITY;
    double MIN_V;
    double MAX_V;
    double DELTA_V;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

};

#endif// __LOOKUP_TABLE_GENERATOR_H
