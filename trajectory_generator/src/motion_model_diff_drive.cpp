#include "trajectory_generator/motion_model_diff_drive.h"

MotionModelDiffDrive::MotionModelDiffDrive()
{
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
    output_s.yaw = atan2(sin(s.yaw), cos(s.yaw));
    output_s.v = v;
    output_s.curvature = curv;
}

void MotionModelDiffDrive::generate_trajectory(const double dt, const double trajectory_length, const double v0, const double k0, const double km, const double kf, std::vector<double>& x_list, std::vector<double>& y_list, std::vector<double>& yaw_list)
{
    const int N = trajectory_length / trajectory_resolution;
    Eigen::Vector3d kk;
    kk << k0, km, kf;

    Eigen::VectorXd coeff0(4);
    Eigen::VectorXd coeff1(4);
    calculate_spline(kk, trajectory_length, coeff0, coeff1); 
    std::vector<double> s_profile;
    for(int i=0;i<N;i++){
        s_profile.push_back(i * trajectory_resolution);
    }
    std::vector<double> curv_profile;
    for(auto s : s_profile){
        double c = 0;
        if(s < trajectory_length / 2.0){
            c = calculate_cubic_function(s, coeff0);
        }else{
            c = calculate_cubic_function(s, coeff1);
        }
        curv_profile.push_back(c);
    }

    State state(0, 0, 0, v0, k0);
    State state_(0, 0, 0, v0, k0);
    x_list.push_back(state.x);
    y_list.push_back(state.y);
    yaw_list.push_back(state.yaw);

    for(int i=0;i<N-1;i++){
        update(state, s_profile[i], curv_profile[i], dt, state_);
        x_list.push_back(state.x);
        y_list.push_back(state.y);
        yaw_list.push_back(state.yaw);
        state = state_;
    }
}

void MotionModelDiffDrive::generate_last_state(const double dt, const double trajectory_length, const double v0, const double k0, const double km, const double kf, Eigen::Vector3d& output)
{
    std::vector<double> x_list;
    std::vector<double> y_list;
    std::vector<double> yaw_list;
    generate_trajectory(dt, trajectory_length, v0, k0, km, kf, x_list, y_list, yaw_list);
    output << x_list.back(), y_list.back(), yaw_list.back();
}

void MotionModelDiffDrive::calculate_spline(const Eigen::Vector3d curv, const double sf, Eigen::VectorXd& coeff0, Eigen::VectorXd& coeff1)
{
    /*
     * curv: k0, km, kf
     * sf: trajectory length [m]
     */
    Eigen::Vector3d x;
    x(0) = 0;
    x(1) = sf / 2.0;
    x(2) = sf;
    Eigen::Vector3d y;
    y = curv; 

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
    a = s.lu().solve(c);
    // ax^3 + bx^2 + cx + d = y
    // coeff: (a, b, c, d)
    coeff0 = a.segment(0, 4);
    coeff1 = a.segment(4, 4);
}

double MotionModelDiffDrive::calculate_cubic_function(const double x, const Eigen::VectorXd& coeff)
{
    return coeff(0) * x * x * x + coeff(1) * x * x + coeff(2) + x + coeff(3);
}
