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
        State(float, float, float, float, float);

        float x;// robot position x
        float y;// robot posiiton y
        float yaw;// robot orientation yaw
        float v;// robot linear velocity
        float curvature;// trajectory curvature
    private:
    };

    class VelocityParams
    {
    public:
        VelocityParams(void);
        VelocityParams(float, float, float, float, float);

        float v0;
        float a0;
        float vt;
        float vf;
        float af;
        float time;
    private:
    };

    class CurvatureParams
    {
    public:
        CurvatureParams(void);
        CurvatureParams(float, float, float, float);

        void calculate_spline(void);

        float k0;
        float km;
        float kf;
        float sf;
        Eigen::Vector3f coeff_0_m;
        Eigen::Vector3f coeff_m_f;
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
        std::vector<Eigen::Vector3f> trajectory;
        std::vector<float> velocities;
        std::vector<float> angular_velocities;
        float cost;
    private:
    };

    void set_param(const float, const float, const float, const float);
    void generate_trajectory(const float, const ControlParams&, Trajectory&);
    void generate_last_state(const float, const float, const VelocityParams&, const float, const float, const float, Eigen::Vector3f&);
    void make_velocity_profile(const float, const VelocityParams&);
    float estimate_driving_time(const ControlParams&);
    void update(const State& s, const float v, const float curv, const float dt, State& output_s)
    {
        float vdt = v * dt;
        output_s.x = s.x + vdt * cos(s.yaw);
        output_s.y = s.y + vdt * sin(s.yaw);
        output_s.yaw = s.yaw + curv * vdt;
        if(output_s.yaw < -M_PI || output_s.yaw > M_PI){
            output_s.yaw = atan2(sin(output_s.yaw), cos(output_s.yaw));
        }
        response_to_control_inputs(s, dt, output_s);
    }

    float calculate_quadratic_function(const float x, const Eigen::Vector3f& coeff)
    {
        return coeff(0) * x * x + coeff(1) * x + coeff(2);
    }

    void response_to_control_inputs(const State& state, const float dt, State& output)
    {
        float _dt = 1.0 / dt;
        float k = state.curvature;
        float _k = output.curvature;
        float dk = (_k - k) * _dt;
        dk = std::max(std::min(dk, MAX_D_CURVATURE), -MAX_D_CURVATURE);

        // adjust output.v
        control_speed(output, output);

        _k += dk * dt;
        output.curvature = std::max(std::min(_k, MAX_CURVATURE), -MAX_CURVATURE);

        float v = state.v;
        float _v = output.v;
        float a = (_v - v) * _dt;
        a = std::max(std::min(a, MAX_ACCELERATION), -MAX_ACCELERATION);
        _v += a * dt;
        output.v = _v;
    }

    void control_speed(const State& state, State& _state)
    {
        // speed control logic
        _state = state;
        float yawrate = _state.curvature * _state.v;
        if(fabs(yawrate) > MAX_YAWRATE){
            _state.v = MAX_YAWRATE / _state.curvature;
        }
    }

private:
    float MAX_YAWRATE;
    float MAX_D_CURVATURE;
    float MAX_CURVATURE;
    float MAX_ACCELERATION;

    std::vector<float> v_profile;
    std::vector<float> s_profile;
};

#endif //__MOTION_MODEL_DIFF_DRIVE_H
