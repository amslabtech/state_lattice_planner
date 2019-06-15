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
    double generate_optimized_trajectory(const Eigen::Vector3d&, const MotionModelDiffDrive::VelocityParams&, const MotionModelDiffDrive::CurvatureParams&, const double, const double, const int, MotionModelDiffDrive::VelocityParams&, MotionModelDiffDrive::CurvatureParams&, std::vector<Eigen::Vector3d>&);
    void get_jacobian(const double, const double, const MotionModelDiffDrive::CurvatureParams&, const Eigen::Vector3d&, Eigen::Matrix3d&);

private:
    MotionModelDiffDrive model;

    Eigen::Vector3d h;
};

#endif //__TRAJECTORY_GENERATOR_DIFF_DRIVE_H
