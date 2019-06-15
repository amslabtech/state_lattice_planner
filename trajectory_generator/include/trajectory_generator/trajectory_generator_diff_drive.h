#ifndef __TRAJECTORY_GENERATOR_DIFF_DRIVE_H
#define __TRAJECTORY_GENERATOR_DIFF_DRIVE_H

#include <ros/ros.h>

#include <Eigen/Dense>

#include <trajectory_generator/motion_model_diff_drive.h>

class TrajectoryGeneratorDiffDrive
{
public:
    TrajectoryGeneratorDiffDrive(void);

    void get_jacobian(const Eigen::Vector3d&, const double, const double, const double, const double, const double, const double, const Eigen::Vector3d&, Eigen::Matrix3d&);

private:
    MotionModelDiffDrive model;
};

#endif //__TRAJECTORY_GENERATOR_DIFF_DRIVE_H
