#ifndef __MOTION_MODEL_DIFF_DRIVE_H
#define __MOTION_MODEL_DIFF_DRIVE_H

#include <ros/ros.h>

#include <Eigen/Dense>

class MotionModelDiffDrive
{
public:
    MotionModelDiffDrive(void);

    class State
    {
    public:
        State(double, double, double, double, double);

        double x;// robot position x
        double y;// robot posiiton y
        double yaw;// robot orientation yaw
        double v;// robot linear velocity
        double curvature;// trajectory curvature
    private:
    };

    class VelocityParams
    {
    public:
        VelocityParams(double, double);

        double v0;
        double time;
    private:
    };

    class CurvatureParams
    {
    public:
        CurvatureParams(double, double, double, double);

        void calculate_spline(void);

        double k0;
        double km;
        double kf;
        double sf;
        Eigen::VectorXd coeff_0_m;
        Eigen::VectorXd coeff_m_f;
    private:
    };

    void set_param(const double, const double, const double, const double, const double);
    void update(const State&, const double, const double, const double, State&);
    void calculate_spline(const CurvatureParams&, Eigen::VectorXd&, Eigen::VectorXd&);
    void generate_trajectory(const double, const double, const CurvatureParams&, std::vector<Eigen::Vector3d>&);
    void generate_last_state(const double, const double, const double, const double, const double, const double, Eigen::Vector3d&);
    double calculate_cubic_function(const double, const Eigen::VectorXd&);

private:
    double trajectory_resolution;
    double target_velocity;
    double max_curvature;
    double max_acceleration;
    double max_d_curvature;
};

#endif //__MOTION_MODEL_DIFF_DRIVE_H
