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
        VelocityParams(void);
        VelocityParams(double, double, double, double, double);

        double v0;
        double a0;
        double vt;
        double vf;
        double af;
        double time;
    private:
    };

    class CurvatureParams
    {
    public:
        CurvatureParams(void);
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

    class ControlParams
    {
    public:
        ControlParams(void);
        ControlParams(const VelocityParams&, const CurvatureParams&);

        VelocityParams vel;
        CurvatureParams curv;
    private:
    };

    class Trajectory
    {
    public:
        Trajectory(void);

        bool operator<(const Trajectory&) const;

        // these vectors must be same size
        std::vector<Eigen::Vector3d> trajectory;
        std::vector<double> velocities;
        std::vector<double> angular_velocities;
        double cost;
    private:
    };

    void set_param(const double);
    void update(const State&, const double, const double, const double, State&);
    void calculate_spline(const CurvatureParams&, Eigen::VectorXd&, Eigen::VectorXd&);
    void generate_trajectory(const double, const ControlParams&, Trajectory&);
    void generate_last_state(const double, const double, const VelocityParams&, const double, const double, const double, Eigen::Vector3d&);
    double calculate_cubic_function(const double, const Eigen::VectorXd&);
    double calculate_quadratic_function(const double, const Eigen::VectorXd&);
    void make_velocity_profile(const double, const VelocityParams&);
    double estimate_driving_time(const ControlParams&);
    void response_to_control_inputs(const State&, const double, State&);
    void control_speed(const State&, const double, State&);

private:
    double MAX_YAWRATE;
    std::vector<double> v_profile;
    std::vector<double> s_profile;
};

#endif //__MOTION_MODEL_DIFF_DRIVE_H
