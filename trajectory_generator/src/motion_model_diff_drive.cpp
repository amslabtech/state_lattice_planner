#include "trajectory_generator/motion_model_diff_drive.h"

MotionModelDiffDrive::MotionModelDiffDrive()
{
    // default setting
    MAX_YAWRATE = 0.8;
    MAX_D_CURVATURE = 2.0;
    MAX_CURVATURE = 1.0;
    MAX_ACCELERATION = 1.0;
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

MotionModelDiffDrive::VelocityParams::VelocityParams(double _v0, double _a0, double _vt, double _vf, double _af)
{
    v0 = _v0;
    a0 = _a0;
    vt = _vt;
    vf = _vf;
    af = _af;
    time = 0;
}

MotionModelDiffDrive::CurvatureParams::CurvatureParams(void)
{
    coeff_0_m = Eigen::VectorXd::Zero(3);
    coeff_m_f = Eigen::VectorXd::Zero(3);
}

MotionModelDiffDrive::CurvatureParams::CurvatureParams(double _k0, double _km, double _kf, double _sf)
{
    k0 = _k0;
    km = _km;
    kf = _kf;
    sf = _sf;
    coeff_0_m = Eigen::VectorXd::Zero(3);
    coeff_m_f = Eigen::VectorXd::Zero(3);
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

bool MotionModelDiffDrive::Trajectory::operator<(const Trajectory& another) const
{
    return cost < another.cost;
}

void MotionModelDiffDrive::set_param(const double max_yawrate, const double max_curvature, const double max_d_curvature, const double max_acceleration)
{
    MAX_YAWRATE = max_yawrate;
    MAX_CURVATURE = max_curvature;
    MAX_D_CURVATURE = max_d_curvature;
    MAX_ACCELERATION = max_acceleration;
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
    response_to_control_inputs(s, dt, output_s);
}

void MotionModelDiffDrive::generate_trajectory(const double dt, const ControlParams& control_param, Trajectory& trajectory)
{
    double start = ros::Time::now().toSec();
    //std::cout << "gen start" << std::endl;
    CurvatureParams curv = control_param.curv;
    VelocityParams vel = control_param.vel;

    vel.time = estimate_driving_time(control_param);
    //std::cout << "estimate time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;

    make_velocity_profile(dt, vel);
    //std::cout << "v prof: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
    //std::cout << vel.v0 << ", " << vel.vt << ", " << vel.vf << ", " << vel.time << ", " << curv.sf << ", " << std::endl;
    const int N = s_profile.size();
    //std::cout << "n: " << N << std::endl;

    curv.calculate_spline();
    //std::cout << "spline: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
    std::vector<double> curv_profile;
    double sf_2 = curv.sf * 0.5;
    for(const auto& s : s_profile){
        double c = 0;
        if(s < sf_2){
            //c = calculate_cubic_function(s, curv.coeff_0_m);
            c = calculate_quadratic_function(s, curv.coeff_0_m);
        }else{
            //c = calculate_cubic_function(s, curv.coeff_m_f);
            c = calculate_quadratic_function(s-sf_2, curv.coeff_m_f);
        }
        curv_profile.push_back(c);
    }
    //std::cout << "curv prof: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
    State state(0, 0, 0, vel.v0, curv.k0);
    State state_(0, 0, 0, vel.v0, curv.k0);
    Eigen::Vector3d pose;
    pose << state.x, state.y, state.yaw;
    trajectory.trajectory.resize(N);
    trajectory.velocities.resize(N);
    trajectory.angular_velocities.resize(N);
    /*
    trajectory.trajectory.push_back(pose);
    trajectory.velocities.push_back(state.v);
    trajectory.angular_velocities.push_back(state.v * state.curvature);
    */
    trajectory.trajectory[0] = pose;
    trajectory.velocities[0] = state.v;
    trajectory.angular_velocities[0] = state.v * state.curvature;
    //std::cout << "prof: " << ros::Time::now().toSec() - start << "[s]" << std::endl;

    start = ros::Time::now().toSec();
    for(int i=1;i<N;i++){
        //double u_start = ros::Time::now().toSec();
        update(state, v_profile[i], curv_profile[i], dt, state_);
        state = state_;
        pose << state.x, state.y, state.yaw;
        /*
        trajectory.trajectory.push_back(pose);
        trajectory.velocities.push_back(state.v);
        trajectory.angular_velocities.push_back(state.v * state.curvature);
        */
        trajectory.trajectory[i] = pose;
        trajectory.velocities[i] = state.v;
        trajectory.angular_velocities[i] = state.v * state.curvature;
        //std::cout << "t" << i << ": " << ros::Time::now().toSec() - u_start << "[s]" << std::endl;
    }
    //std::cout << "gen t: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
}

void MotionModelDiffDrive::generate_last_state(const double dt, const double trajectory_length, const VelocityParams& vel, const double k0, const double km, const double kf, Eigen::Vector3d& output)
{
    //std::cout << "--- generate last state ---" << std::endl;
    double start = ros::Time::now().toSec();
    Trajectory trajectory;
    CurvatureParams curv(k0, km, kf, trajectory_length);
    generate_trajectory(dt, ControlParams(vel, curv), trajectory);
    output << trajectory.trajectory.back();
    //std::cout << "gls time: " << ros::Time::now().toSec() - start << std::endl;
}

void MotionModelDiffDrive::CurvatureParams::calculate_spline(void)
{
    //std::cout << "spline" << std::endl;
    //double start = ros::Time::now().toSec();
    /*
    Eigen::Vector3d x;
    x(0) = 0;
    x(1) = sf * 0.5;
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
    std::cout << "spline mat: " << ros::Time::now().toSec() - start << "[s]" << std::endl;

    Eigen::VectorXd c(8);
    c << y(0), y(1), y(1), y(2), 0, 0, 0, 0;

    Eigen::VectorXd a(8);
    //a = s.inverse() * c; inverse() is too slow!!!
    a = s.lu().solve(c);
    std::cout << "spline inv: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
    // ax^3 + bx^2 + cx + d = y
    // coeff: (a, b, c, d)
    coeff_0_m = a.segment(0, 4);
    coeff_m_f = a.segment(4, 4);
    */
    // 2d spline interpolation
    Eigen::Vector3d x(0, sf / 2.0, sf);
    Eigen::Vector3d y(k0, km, kf);
    Eigen::Matrix3d s;
    s << 2 * (x(1) - x(0)),             x(1), -x(1),
         x(1) * x(1),                   x(1), 0,
         (x(2) - x(1)) * (x(2) - x(1)), 0,    x(2) - x(1);
    /*
    s << x(1) * x(1), x(1), 0,
         x(1) * x(1), 0,    x(1),
         x(2),        1,    -1;
    */
    Eigen::Vector3d c(0, y(1) - y(0), y(2) - y(1));
    //Eigen::Vector3d c(y(1) - y(0), y(2) - y(1), 0);
    Eigen::Vector3d a = s.inverse() * c;
    coeff_0_m << a(0), a(1), y(0);
    //std::cout << coeff_0_m << std::endl;
    coeff_m_f << a(0), a(2), y(1);
    //std::cout << coeff_m_f << std::endl;
    //std::cout << "spline end: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
}

inline double MotionModelDiffDrive::calculate_cubic_function(const double x, const Eigen::VectorXd& coeff)
{
    return coeff(0) * x * x * x + coeff(1) * x * x + coeff(2) * x + coeff(3);
}

inline double MotionModelDiffDrive::calculate_quadratic_function(const double x, const Eigen::VectorXd& coeff)
{
    return coeff(0) * x * x + coeff(1) * x + coeff(2);
}

void MotionModelDiffDrive::make_velocity_profile(const double dt, const VelocityParams& v_param)
{
    /*
     *  trapezoid control
     */

    /***************************************
         vt  ________________
           /|                |\
          / |                | \
     v0  /  |                |  \ vf
          a0      a=0         af

    ***************************************/
    int size = v_param.time / dt;
    v_profile.resize(size);
    s_profile.resize(size);

    double s = 0;
    double t = 0;
    for(int i=0;i<size;++i){
        // acceleration time
        double ta = fabs((v_param.vt - v_param.v0) / v_param.a0);
        // deceleration time
        double td = v_param.time - fabs((v_param.vf - v_param.vt) / v_param.af);

        double v = 0;
        if(t < 0){
            v = v_param.v0;
        }else if(t < ta){
            if(v_param.v0 + v_param.a0 * t < v_param.vt){
                v = v_param.v0 + v_param.a0 * t;
            }else{
                v = v_param.vt;
            }
        }else if(ta <= t && t < td){
            v = v_param.vt;
        }else if(td <= t && t < v_param.time){
            if(v_param.vt - v_param.af * (t - td) > v_param.vf){
                v = v_param.vt - v_param.af * (t - td);
            }else{
                v = v_param.vt;
            }
        }else{
            v = v_param.vf;
        }
        v_profile[i] = v;
        s += fabs(v) * dt;
        s_profile[i] = s;
        t += dt;
    }
}

double MotionModelDiffDrive::estimate_driving_time(const ControlParams& control)
{
    // acceleration time
    double t0 = fabs((control.vel.vt - control.vel.v0) / control.vel.a0);
    // deceleration time
    double td = fabs((control.vel.vf - control.vel.vt) / control.vel.af);
    //std::cout << t0 << ", " << td << std::endl;

    double s0 = 0.5 * fabs(control.vel.vt + control.vel.v0) * t0;
    double sd = 0.5 * fabs(control.vel.vt + control.vel.vf) * td;
    //std::cout << s0 << ", " << sd << std::endl;

    double st = control.curv.sf - s0 - sd;
    double tt = st / fabs(control.vel.vt);
    //std::cout << st << ", " << tt << std::endl;
    double driving_time = t0 + tt + td;
    return driving_time;
}

void MotionModelDiffDrive::response_to_control_inputs(const State& state, const double dt, State& output)
{
    double k = state.curvature;
    double _k = output.curvature;
    double dk = (_k - k) / dt;
    dk = std::max(std::min(dk, MAX_D_CURVATURE), -MAX_D_CURVATURE);

    // adjust output.v
    control_speed(output, output);

    _k += dk * dt;
    output.curvature = std::max(std::min(_k, MAX_CURVATURE), -MAX_CURVATURE);

    double v = state.v;
    double _v = output.v;
    double a = (_v - v) / dt;
    a = std::max(std::min(a, MAX_ACCELERATION), -MAX_ACCELERATION);
    _v += a * dt;
    output.v = _v;
}

void MotionModelDiffDrive::control_speed(const State& state, State& _state)
{
    // speed control logic
    _state = state;
    double yawrate = _state.curvature * _state.v;
    if(fabs(yawrate) > MAX_YAWRATE){
        _state.v = MAX_YAWRATE / _state.curvature;
    }
}
