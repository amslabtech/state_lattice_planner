#include "trajectory_generator/motion_model_diff_drive.h"

MotionModelDiffDrive::MotionModelDiffDrive()
{
    // default setting
    trajectory_resolution = 0.05;
    target_velocity = 0.5;
    max_curvature = 0.5;
    max_acceleration = 1.0;
    max_d_curvature = 0.5;
}

MotionModelDiffDrive::State::State(double _x, double _y, double _yaw, double _v, double _curvature)
{
    x = _x;
    y = _y;
    yaw = _yaw;
    v = _v;
    curvature = _curvature;
}

MotionModelDiffDrive::VelocityParams::VelocityParams(void)
{

}

MotionModelDiffDrive::VelocityParams::VelocityParams(double _v0, double _time)
{
    v0 = _v0;
    time = _time;
}

MotionModelDiffDrive::CurvatureParams::CurvatureParams(void)
{
    coeff_0_m = Eigen::VectorXd::Zero(4);
    coeff_m_f = Eigen::VectorXd::Zero(4);
}

MotionModelDiffDrive::CurvatureParams::CurvatureParams(double _k0, double _km, double _kf, double _sf)
{
    k0 = _k0;
    km = _km;
    kf = _kf;
    sf = _sf;
    coeff_0_m = Eigen::VectorXd::Zero(4);
    coeff_m_f = Eigen::VectorXd::Zero(4);
}

MotionModelDiffDrive::ControlParams::ControlParams(void)
{

}

MotionModelDiffDrive::ControlParams::ControlParams(const VelocityParams& _vel, const CurvatureParams& _curv)
{
    vel = _vel;
    curv = _curv;
}

MotionModelDiffDrive::Trajectory::Trajectory(void)
{

}

void MotionModelDiffDrive::set_param(const double trajectory_resolution_, const double target_velocity_, const double max_curvature_, const double max_acceleration_, const double max_d_curvature_)
{
    trajectory_resolution = trajectory_resolution_;
    target_velocity = target_velocity_;
    max_curvature = max_curvature_;
    max_acceleration = max_acceleration_;
    max_d_curvature = max_d_curvature_;
}

void MotionModelDiffDrive::update(const State& s, const double v, const double curv, const double dt, State& output_s)
{
    output_s = s;
    output_s.x += v * cos(s.yaw) * dt;
    output_s.y += v * sin(s.yaw) * dt;
    output_s.yaw += curv * v * dt;
    output_s.yaw = atan2(sin(output_s.yaw), cos(output_s.yaw));
    output_s.v = v;
    output_s.curvature = curv;
}

void MotionModelDiffDrive::generate_trajectory(const double dt, const double v0, const CurvatureParams& curv, Trajectory& trajectory)
{
    const int N = curv.sf / trajectory_resolution;

    CurvatureParams _curv = curv;
    _curv.calculate_spline();
    std::vector<double> s_profile;
    for(int i=0;i<N;i++){
        s_profile.push_back(i * trajectory_resolution);
    }
    std::vector<double> curv_profile;
    for(auto s : s_profile){
        double c = 0;
        if(s < _curv.sf / 2.0){
            c = calculate_cubic_function(s, _curv.coeff_0_m);
        }else{
            c = calculate_cubic_function(s, _curv.coeff_m_f);
        }
        curv_profile.push_back(c);
    }
    //std::cout << "N: " << N << std::endl;
    State state(0, 0, 0, v0, _curv.k0);
    State state_(0, 0, 0, v0, _curv.k0);
    Eigen::Vector3d pose;
    pose << state.x, state.y, state.yaw;
    trajectory.trajectory.push_back(pose);
    trajectory.velocities.push_back(state.v);
    trajectory.angular_velocities.push_back(state.v * state.curvature);

    for(int i=1;i<N;i++){
        update(state, (s_profile[i]-s_profile[i-1])/dt, curv_profile[i], dt, state_);
        state = state_;
        pose << state.x, state.y, state.yaw;
        trajectory.trajectory.push_back(pose);
        trajectory.velocities.push_back(state.v);
        trajectory.angular_velocities.push_back(state.v * state.curvature);
    }
}

void MotionModelDiffDrive::generate_last_state(const double dt, const double trajectory_length, const double v0, const double k0, const double km, const double kf, Eigen::Vector3d& output)
{
    Trajectory trajectory;
    CurvatureParams curv(k0, km, kf, trajectory_length);
    generate_trajectory(dt, v0, curv, trajectory);
    output << trajectory.trajectory.back();
}

void MotionModelDiffDrive::CurvatureParams::calculate_spline(void)
{
    Eigen::Vector3d x;
    x(0) = 0;
    x(1) = sf / 2.0;
    x(2) = sf;
    Eigen::Vector3d y;
    y << k0, km, kf;

    // cubic spline interpolation
    Eigen::MatrixXd s(8, 8);
    s << x(0) * x(0) * x(0), x(0) * x(0), x(0), 1, 0, 0, 0, 0,
         x(1) * x(1) * x(1), x(1) * x(1), x(1), 1, 0, 0, 0, 0,
         0, 0, 0, 0, x(1) * x(1) * x(1), x(1) * x(1), x(1), 1,
         0, 0, 0, 0, x(2) * x(2) * x(2), x(2) * x(2), x(2), 1,
         3 * x(1) * x(1), 2 * x(1), 1, 0, -3 * x(1) * x(1), -2 * x(1), -1, 0,
         6 * x(1), 2, 0, 0, -6 * x(1), -2, 0, 0,
         6 * x(0), 2, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 6 * x(2), 2, 0, 0;

    Eigen::VectorXd c(8);
    c << y(0), y(1), y(1), y(2), 0, 0, 0, 0;

    Eigen::VectorXd a(8);
    a = s.inverse() * c;
    // ax^3 + bx^2 + cx + d = y
    // coeff: (a, b, c, d)
    coeff_0_m = a.segment(0, 4);
    coeff_m_f = a.segment(4, 4);
}

double MotionModelDiffDrive::calculate_cubic_function(const double x, const Eigen::VectorXd& coeff)
{
    return coeff(0) * x * x * x + coeff(1) * x * x + coeff(2) * x + coeff(3);
}
