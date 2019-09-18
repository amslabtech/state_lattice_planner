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
        double omega;// robot angular velocity
    private:
    };

    class VelocityParams
    {
    public:
        VelocityParams(void);
        VelocityParams(double, double, double, double, double);// v0, a0, vt, vf, af

        double v0;
        double a0;
        double vt;
        double vf;
        double af;
        double time;
    private:
    };

    class AngularVelocityParams
    {
    public:
        AngularVelocityParams(void);
        AngularVelocityParams(double, double, double, double);

        void calculate_spline(void);

        double k0;
        double km;
        double kf;
        double sf;
        Eigen::Matrix<double, 4, 1> coeff_0_m;
        Eigen::Matrix<double, 4, 1> coeff_m_f;
    private:
    };

    class ControlParams
    {
    public:
        ControlParams(void);
        ControlParams(const VelocityParams&, const AngularVelocityParams&);

        VelocityParams vel;
        AngularVelocityParams omega;
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

    void set_param(const double, const double, const double, const double, const double, const double, const double);
    void generate_trajectory(const double, const ControlParams&, Trajectory&);
    void generate_last_state(const double, const double, const VelocityParams&, const double, const double, const double, Eigen::Vector3d&);
    void make_velocity_profile(const double, const VelocityParams&);
    double estimate_driving_time(const ControlParams&);
    void update(const State&, const double, const double, const double, State&);
    double calculate_quadratic_function(const double, const Eigen::Vector3d&);
    double calculate_cubic_function(const double, const Eigen::Matrix<double, 4, 1>&);
    void response_to_control_inputs(const State&, const double, State&);
    void control_speed(const State& state, State& _state);

private:
    double MAX_YAWRATE;
    double MAX_D_CURVATURE;
    double MAX_CURVATURE;
    double MAX_ACCELERATION;
    double MAX_WHEEL_ANGULAR_VELOCITY;
    double WHEEL_RADIUS;
    double TREAD;

    std::vector<double> v_profile;
    std::vector<double> s_profile;
};

#endif //__MOTION_MODEL_DIFF_DRIVE_H
