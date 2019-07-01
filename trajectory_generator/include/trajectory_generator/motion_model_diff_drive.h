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
        Eigen::Vector3d coeff_0_m;
        Eigen::Vector3d coeff_m_f;
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

    void set_param(const double, const double, const double, const double);
    void generate_trajectory(const double, const ControlParams&, Trajectory&);
    void generate_last_state(const double, const double, const VelocityParams&, const double, const double, const double, Eigen::Vector3d&);
    void make_velocity_profile(const double, const VelocityParams&);
    double estimate_driving_time(const ControlParams&);
    void update(const State& s, const double v, const double curv, const double dt, State& output_s)
    {
        double vdt = v * dt;
        output_s.x = s.x + vdt * cos(s.yaw);
        output_s.y = s.y + vdt * sin(s.yaw);
        output_s.yaw = s.yaw + curv * vdt;
        if(output_s.yaw < -M_PI || output_s.yaw > M_PI){
            output_s.yaw = atan2(sin(output_s.yaw), cos(output_s.yaw));
        }
        response_to_control_inputs(s, dt, output_s);
    }

    double calculate_quadratic_function(const double x, const Eigen::Vector3d& coeff)
    {
        return coeff(0) * x * x + coeff(1) * x + coeff(2);
    }

    void response_to_control_inputs(const State& state, const double dt, State& output)
    {
        double _dt = 1.0 / dt;
        double k = state.curvature;
        double _k = output.curvature;
        double dk = (_k - k) * _dt;
        dk = std::max(std::min(dk, MAX_D_CURVATURE), -MAX_D_CURVATURE);

        // adjust output.v
        control_speed(output, output);

        _k += dk * dt;
        output.curvature = std::max(std::min(_k, MAX_CURVATURE), -MAX_CURVATURE);

        double v = state.v;
        double _v = output.v;
        double a = (_v - v) * _dt;
        a = std::max(std::min(a, MAX_ACCELERATION), -MAX_ACCELERATION);
        _v += a * dt;
        output.v = _v;
    }

    void control_speed(const State& state, State& _state)
    {
        // speed control logic
        _state = state;
        double yawrate = _state.curvature * _state.v;
        if(fabs(yawrate) > MAX_YAWRATE){
            _state.v = MAX_YAWRATE / _state.curvature;
        }
    }

private:
    double MAX_YAWRATE;
    double MAX_D_CURVATURE;
    double MAX_CURVATURE;
    double MAX_ACCELERATION;

    std::vector<double> v_profile;
    std::vector<double> s_profile;
};

#endif //__MOTION_MODEL_DIFF_DRIVE_H
