#ifndef __TRAJECTORY_GENERATOR_DIFF_DRIVE_H
#define __TRAJECTORY_GENERATOR_DIFF_DRIVE_H

#include <ros/ros.h>

#include <Eigen/Dense>

#include <trajectory_generator/motion_model_diff_drive.h>

class TrajectoryGeneratorDiffDrive
{
public:
    TrajectoryGeneratorDiffDrive(void);

    void set_optimization_param(const float, const float, const float);
    void set_motion_param(const float, const float, const float, const float);
    float generate_optimized_trajectory(const Eigen::Vector3f&, const MotionModelDiffDrive::ControlParams&, const float, const float, const int, MotionModelDiffDrive::ControlParams&, MotionModelDiffDrive::Trajectory&);
    void get_jacobian(const float, const MotionModelDiffDrive::ControlParams&, const Eigen::Vector3f&, Eigen::Matrix3f&);

private:
    MotionModelDiffDrive model;

    Eigen::Vector3f h;
};

#endif //__TRAJECTORY_GENERATOR_DIFF_DRIVE_H
