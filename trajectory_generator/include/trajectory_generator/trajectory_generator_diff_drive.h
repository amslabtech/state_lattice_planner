#ifndef __TRAJECTORY_GENERATOR_DIFF_DRIVE_H
#define __TRAJECTORY_GENERATOR_DIFF_DRIVE_H

#include <ros/ros.h>

#include <Eigen/Dense>

#include <trajectory_generator/motion_model_diff_drive.h>

class TrajectoryGeneratorDiffDrive
{
public:
    TrajectoryGeneratorDiffDrive(void);

    void set_param(const double, const double, const double);
    double generate_optimized_trajectory(const Eigen::Vector3d&, const MotionModelDiffDrive::ControlParams&, const double, const double, const int, MotionModelDiffDrive::ControlParams&, MotionModelDiffDrive::Trajectory&);
    void get_jacobian(const double, const double, const MotionModelDiffDrive::CurvatureParams&, const Eigen::Vector3d&, Eigen::Matrix3d&);

private:
    MotionModelDiffDrive model;

    Eigen::Vector3d h;
};

#endif //__TRAJECTORY_GENERATOR_DIFF_DRIVE_H
