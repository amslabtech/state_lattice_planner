/**
 * @file motion_model_diff_drive.h
 * @author AMSL
 */
#ifndef __MOTION_MODEL_DIFF_DRIVE_H
#define __MOTION_MODEL_DIFF_DRIVE_H

#include <iostream>
#include <vector>
#include <chrono>

#include <Eigen/Dense>

/**
 * @brief Trajectory generation with motion model of differential drive robots
 */
class MotionModelDiffDrive
{
public:
    /**
     * @brief Constructor
     */
    MotionModelDiffDrive(void);

    /**
     * @brief Class representing robot state
     */
    class State
    {
    public:
        /**
         * @brief Constructor
         * @param[in] _x x coordinate in robot frame [m]
         * @param[in] _y y coordinate in robot frame [m]
         * @param[in] _yaw Orientation [rad]
         * @param[in] _velocity Linear velocity [m/s]
         * @param[in] _omega Angular velocity [rad/s]
         */
        State(double, double, double, double, double);

        double x;// robot position x
        double y;// robot posiiton y
        double yaw;// robot orientation yaw
        double v;// robot linear velocity
        double omega;// robot angular velocity
    private:
    };

    /**
     * @brief Class representing velocity profile for trajectories
     */
    class VelocityParams
    {
    public:
        /**
         * @brief Constructor
         */
        VelocityParams(void);
        /**
         * @brief Constructor
         * @param[in] _v0 Initial velocity [m/s]
         * @param[in] _a0 Initial linear acceleration [m/ss]
         * @param[in] _vt Target velocity [m/s]
         * @param[in] _vf Terminal velocity [m/s]
         * @param[in] _af Terminal acceleration [m/ss]
         */
        VelocityParams(double, double, double, double, double);// v0, a0, vt, vf, af

        double v0;
        double a0;
        double vt;
        double vf;
        double af;
        double time;
    private:
    };

    /**
     * @brief Class representing yawrate profile for trajectories
     */
    class AngularVelocityParams
    {
    public:
        /**
         * @brief Constructor
         */
        AngularVelocityParams(void);
        /**
         * @brief Constructor
         * @param[in] _k0 Initial angular velocity [rad/s]
         * @param[in] _km Intermediate angular velocity [rad/s]
         * @param[in] _kf Terminal angular velocity [rad/s]
         * @param[in] _sf Length of trajectory [m]
         */
        AngularVelocityParams(double, double, double, double);

        /**
         * @brief Calculate 3D spline parameters from k0, km, kf, sf. #coefficients will be to the parameters
         * @param[in] ratio Ratio of sf (default: 0.5)
         */
        void calculate_spline(double ratio=0.5);

        double k0;
        double km;
        double kf;
        double sf;
        /**
         * @brief (a, b, c, d) <- ax^3+bx^2+cx+d
         */
        std::vector<Eigen::Vector4d> coefficients;
    private:
    };

    /**
     * @brief Class containing VelocityParams and AngularVelocityParams
     */
    class ControlParams
    {
    public:
        /**
         * @brief Constructor
         */
        ControlParams(void);
        /**
         * @brief Constructor
         * @param[in] _vel Input VelocityParams
         * @param[in] _omega Input AngularVelocityParams
         */
        ControlParams(const VelocityParams&, const AngularVelocityParams&);

        VelocityParams vel;
        AngularVelocityParams omega;
    private:
    };

    /**
     * @brief Class representing a trajectory
     */
    class Trajectory
    {
    public:
        /**
         * @brief Constructor
         */
        Trajectory(void);

        // these vectors must be same size
        std::vector<Eigen::Vector3d> trajectory;
        std::vector<double> velocities;
        std::vector<double> angular_velocities;
        double cost;
    private:
    };

    /**
     * @brief Set motion parameters
     * @param[in] max_yawrate [rad/s]
     * @param[in] max_d_yawrate [rad/ss]
     * @param[in] max_acceleration [m/ss]
     * @param[in] max_wheel_angular_velocity [rad/s]
     * @param[in] wheel_radius [m]
     * @param[in] tread [m]
     */
    void set_param(const double, const double, const double, const double, const double, const double);
    /**
     * @brief Generate a trajectory from given control parameters
     * @param[in] dt Time interval between each pose in the trajectory [s]
     * @param[in] control_param Control parameters
     * @param[out] trajectory Generated trajectory
     */
    void generate_trajectory(const double, const ControlParams&, Trajectory&);
    /**
     * @brief Generate a state at the end of a trajectory from given control parameters
     * @param[in] dt Time interval between each pose in the trajectory [s]
     * @param[in] trajectory_length Desired length of trajecotry [m]
     * @param[in] _vel Velocity profile for the trajectory
     * @param[in] k0, km, kf Angular velocity in the trajectory
     * @param[out] output The last state of generated trajectory
     */
    void generate_last_state(const double, const double, const VelocityParams&, const double, const double, const double, Eigen::Vector3d&);
    /**
     * @brief Make veloicity profile from given velocity parameter
     * @param[in] dt Time interval between each pose in the trajectory [s]
     * @param[in] v_param Velocity parameter
     */
    void make_velocity_profile(const double, const VelocityParams&);
    /**
     * @brief Estimate driveing time from given control parameter
     * @param[in] control
     */
    double estimate_driving_time(const ControlParams&);
    /**
     * @brief Update state
     * @param[in] s State(t)
     * @param[in] v Velocity [m/s]
     * @param[in] omega Yawrate [rad/s]
     * @param[in] dt Step time [s]
     * @param[out] output_s Next state
     */
    void update(const State&, const double, const double, const double, State&);
    /**
     * @brief Calculate ax^2 + bx + c
     * @param[in] x
     * @param[in] coeff Coefficients (a, b, c)
     * @retval output ax^2 + bx + c
     */
    double calculate_quadratic_function(const double, const Eigen::Vector3d&);
    /**
     * @brief Calculate ax^3 + bx^2 + cx + d
     * @param[in] x
     * @param[in] coeff Coefficients (a, b, c, d)
     * @retval output ax^3 + bx^2 + cx + d
     */
    double calculate_cubic_function(const double, const Eigen::Vector4d&);
    /**
     * @brief Limit motion
     * @param[in] state Current state
     * @param[in] dt [s]
     * @param[out] output Next state
     */
    void response_to_control_inputs(const State&, const double, State&);
    /**
     * @brief Modify speed
     * @param[in] state Current state
     * @param[out] output Next state
     */
    void control_speed(const State& state, State& _state);

private:
    double MAX_YAWRATE;
    double MAX_D_YAWRATE;
    double MAX_ACCELERATION;
    double MAX_WHEEL_ANGULAR_VELOCITY;
    double WHEEL_RADIUS;
    double TREAD;

    std::vector<double> v_profile;
    std::vector<double> s_profile;

    double ratio;
};

#endif //__MOTION_MODEL_DIFF_DRIVE_H
