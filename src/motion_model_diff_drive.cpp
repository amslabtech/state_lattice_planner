#include "trajectory_generator/motion_model_diff_drive.h"

MotionModelDiffDrive::MotionModelDiffDrive()
{
    // default setting
    MAX_YAWRATE = 0.8;
    MAX_D_YAWRATE = 2.0;
    MAX_ACCELERATION = 1.0;
    WHEEL_RADIUS = 0.125;
    TREAD = 0.5;
    MAX_WHEEL_ANGULAR_VELOCITY = 11.6;

    ratio = 0.50;
}

MotionModelDiffDrive::State::State(double _x, double _y, double _yaw, double _v, double _omega)
{
    x = _x;
    y = _y;
    yaw = _yaw;
    v = _v;
    omega = _omega;
}

MotionModelDiffDrive::VelocityParams::VelocityParams(void)
{

}

MotionModelDiffDrive::VelocityParams::VelocityParams(double _v0, double _a0, double _vt, double _vf, double _af)
{
    v0 = _v0;
    vt = _vt;
    vf = _vf;
    a0 = v0 < vt ? fabs(_a0) : -fabs(_a0);
    // std::cout << _a0 << ", " << a0 << std::endl;
    // a0 = _a0;
    af = vt > vf ? fabs(_af) : -fabs(_af);
    // std::cout << _af << ", " << af << std::endl;
    // af = _af;
    time = 0;
}

MotionModelDiffDrive::AngularVelocityParams::AngularVelocityParams(void)
{
    coefficients.resize(2);
    for(auto c : coefficients){
        c = Eigen::Vector4d::Zero();
    }
}

MotionModelDiffDrive::AngularVelocityParams::AngularVelocityParams(double _k0, double _km, double _kf, double _sf)
{
    k0 = _k0;
    km = _km;
    kf = _kf;
    sf = _sf;
    coefficients.resize(2);
    for(auto c : coefficients){
        c = Eigen::Vector4d::Zero();
    }
}

MotionModelDiffDrive::ControlParams::ControlParams(void)
{

}

MotionModelDiffDrive::ControlParams::ControlParams(const VelocityParams& _vel, const AngularVelocityParams& _omega)
{
    vel = _vel;
    omega = _omega;
}

MotionModelDiffDrive::Trajectory::Trajectory(void)
{

}

void MotionModelDiffDrive::set_param(const double max_yawrate, const double max_d_yawrate, const double max_acceleration, const double max_wheel_angular_velocity, const double wheel_radius, const double tread)
{
    MAX_YAWRATE = max_yawrate;
    MAX_D_YAWRATE = max_d_yawrate;
    MAX_ACCELERATION = max_acceleration;
    MAX_WHEEL_ANGULAR_VELOCITY = max_wheel_angular_velocity;
    WHEEL_RADIUS = wheel_radius;
    TREAD = TREAD;
}

void MotionModelDiffDrive::update(const State& s, const double v, const double omega, const double dt, State& output_s)
{
    output_s.v = v;
    output_s.omega = omega;
    response_to_control_inputs(s, dt, output_s);

    output_s.x = s.x + output_s.v * dt * cos(s.yaw);
    output_s.y = s.y + output_s.v * dt * sin(s.yaw);
    output_s.yaw = s.yaw + output_s.omega * dt;
    if(output_s.yaw < -M_PI || output_s.yaw > M_PI){
        output_s.yaw = atan2(sin(output_s.yaw), cos(output_s.yaw));
    }
}

double MotionModelDiffDrive::calculate_quadratic_function(const double x, const Eigen::Vector3d& coeff)
{
    return coeff(0) * x * x + coeff(1) * x + coeff(2);
}

double MotionModelDiffDrive::calculate_cubic_function(const double x, const Eigen::Vector4d& coeff)
{
    return coeff(0) * x * x * x + coeff(1) * x * x + coeff(2) * x + coeff(3);
}

void MotionModelDiffDrive::response_to_control_inputs(const State& state, const double dt, State& output)
{
    double _dt = 1.0 / dt;
    double k = state.omega;
    double _k = output.omega;
    double dk = (_k - k) * _dt;
    dk = std::max(std::min(dk, MAX_D_YAWRATE), -MAX_D_YAWRATE);

    _k = k + dk * dt;
    output.omega = std::max(std::min(_k, MAX_YAWRATE), -MAX_YAWRATE);

    // adjust output.v
    control_speed(output, output);

    double v = state.v;
    double _v = output.v;
    double a = (_v - v) * _dt;
    a = std::max(std::min(a, MAX_ACCELERATION), -MAX_ACCELERATION);
    output.v = v + a * dt;

    // additional omega limitation
    // double yawrate = output.v * output.omega;
    // if(fabs(yawrate) > MAX_YAWRATE){
    //     output.omega = MAX_YAWRATE / fabs(output.v) * (output.omega > 0 ? 1 : -1);
    // }
}

void MotionModelDiffDrive::control_speed(const State& state, State& _state)
{
    // speed control logic
    _state = state;
    _state.v = WHEEL_RADIUS * std::min(fabs(_state.v) / WHEEL_RADIUS, MAX_WHEEL_ANGULAR_VELOCITY - 0.5 * fabs(_state.omega) * TREAD / WHEEL_RADIUS) * (_state.v >= 0.0 ? 1 : -1);

    // double yawrate = _state.omega * _state.v;
    // if(fabs(yawrate) > MAX_YAWRATE){
    //     _state.v = MAX_YAWRATE / fabs(_state.omega) * (state.v > 0 ? 1 : -1);
    // }
}

void MotionModelDiffDrive::generate_trajectory(const double dt, const ControlParams& control_param, Trajectory& trajectory)
{
    // std::cout << "gen start" << std::endl;
    auto start = std::chrono::system_clock::now();
    AngularVelocityParams omega = control_param.omega;
    VelocityParams vel = control_param.vel;

    vel.time = estimate_driving_time(control_param);
    // std::cout << "driving time: " << vel.time << "[s]" << std::endl;
    auto time = std::chrono::system_clock::now();
    double elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(time - start).count();
    // std::cout << "estimate time: " << elapsed_time << "[s]" << std::endl;

    make_velocity_profile(dt, vel);
    time = std::chrono::system_clock::now();
    elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(time - start).count();
    // std::cout << "v prof: " << elapsed_time << "[s]" << std::endl;
    // std::cout << vel.v0 << ", " << vel.vt << ", " << vel.vf << ", " << vel.time << ", " << omega.sf << ", " << std::endl;

    omega.calculate_spline(ratio);
    const int N = s_profile.size();
    // std::cout << "n: " << N << std::endl;
    if(N == 0){
        return;
    }
    double sf_2 = omega.sf * ratio;

    State state(0, 0, 0, vel.v0, omega.k0);
    State state_(0, 0, 0, vel.v0, omega.k0);
    Eigen::Vector3d pose;
    pose << state.x, state.y, state.yaw;
    trajectory.trajectory.resize(N);
    trajectory.velocities.resize(N);
    trajectory.angular_velocities.resize(N);
    trajectory.trajectory[0] = pose;
    trajectory.velocities[0] = state.v;
    // trajectory.angular_velocities[0] = state.v * state.omega;
    trajectory.angular_velocities[0] = state.omega;

    for(int i=1;i<N;i++){
        double s = s_profile[i];
        double k = 0;
        if(s < sf_2){
            k = calculate_cubic_function(s, omega.coefficients[0]);
        }else{
            k = calculate_cubic_function(s, omega.coefficients[1]);
        }
        update(state, v_profile[i], k, dt, state_);
        state = state_;
        pose << state.x, state.y, state.yaw;
        trajectory.trajectory[i] = pose;
        trajectory.velocities[i] = state.v;
        // trajectory.angular_velocities[i] = state.v * state.omega;
        trajectory.angular_velocities[i] = state.omega;
    }
    time = std::chrono::system_clock::now();
    elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(time - start).count();
    // std::cout << "gen t: " << elapsed_time << "[s]" << std::endl;
}

void MotionModelDiffDrive::generate_last_state(const double dt, const double trajectory_length, const VelocityParams& _vel, const double k0, const double km, const double kf, Eigen::Vector3d& output)
{
    // std::cout << "--- generate last state ---" << std::endl;
    // std::cout << k0 << ", " << km << ", " << kf << ", " << trajectory_length << std::endl;
    AngularVelocityParams omega(k0, km, kf, trajectory_length);
    VelocityParams vel = _vel;

    vel.time = estimate_driving_time(ControlParams(vel, omega));

    make_velocity_profile(dt, vel);

    omega.calculate_spline(ratio);
    const int N = s_profile.size();
    double sf_2 = omega.sf * ratio;
    State state(0, 0, 0, vel.v0, omega.k0);
    output << state.x, state.y, state.yaw;
    for(int i=1;i<N;i++){
        double s = s_profile[i];
        double k = 0;
        if(s < sf_2){
            k = calculate_cubic_function(s, omega.coefficients[0]);
        }else{
            k = calculate_cubic_function(s, omega.coefficients[1]);
        }
        update(state, v_profile[i], k, dt, state);
    }
    output << state.x, state.y, state.yaw;
}

void MotionModelDiffDrive::AngularVelocityParams::calculate_spline(double ratio)
{
    // std::cout << "spline" << std::endl;
    // 3d spline interpolation
    Eigen::Vector3d x(0, sf * ratio, sf);
    Eigen::Vector3d y(k0, km, kf);
    Eigen::Matrix<double, 8, 8> s;
    s << x(0) * x(0) * x(0), x(0) * x(0), x(0), 1, 0, 0, 0, 0,
         x(1) * x(1) * x(1), x(1) * x(1), x(1), 1, 0, 0, 0, 0,
         0, 0, 0, 0, x(1) * x(1) * x(1), x(1) * x(1) , x(1), 1,
         0, 0, 0, 0, x(2) * x(2) * x(2), x(2) * x(2) , x(2), 1,
         3 * x(1) * x(1), 2 * x(1), 1, 0, -3 * x(1) * x(1), -2 * x(1), -1, 0,
         6 * x(1), 2, 0, 0, -6 * x(1), -2, 0, 0,
         6 * x(0), 2, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 6 * x(2), 2, 0, 0;
    Eigen::VectorXd c = Eigen::VectorXd::Zero(8);
    c << y(0), y(1), y(1), y(2), 0, 0, 0, 0;
    Eigen::VectorXd a = s.inverse() * c;
    coefficients[0] = a.segment(0, 4);
    coefficients[1] = a.segment(4, 4);
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
    int size = v_param.time / dt + 1;
    // std::cout << "size: " << size << std::endl;
    v_profile.resize(size);
    s_profile.resize(size);

    double s = 0;
    double t = 0;
    for(int i=0;i<size;++i){
        // acceleration time
        double ta = fabs((v_param.vt - v_param.v0) / v_param.a0);
        // std::cout << "ta: " << ta << std::endl;
        // deceleration time
        double td = v_param.time - fabs((v_param.vf - v_param.vt) / v_param.af);
        // std::cout << "td: " << ta << std::endl;

        double v = 0;
        if(t < 0){
            v = v_param.v0;
        }else if(t < ta){
            if(fabs(v_param.v0 + v_param.a0 * t) < fabs(v_param.vt)){
                v = v_param.v0 + v_param.a0 * t;
            }else{
                v = v_param.vt;
            }
        }else if(ta <= t && t < td){
            v = v_param.vt;
        }else if(td <= t && t < v_param.time){
            if(fabs(v_param.vt - v_param.af * (t - td)) > fabs(v_param.vf)){
                v = v_param.vt - v_param.af * (t - td);
            }else{
                v = v_param.vt;
            }
        }else{
            v = v_param.vf;
        }
        v_profile[i] = v;
        // std::cout << "v: " << v << std::endl;
        s_profile[i] = s;
        // std::cout << "s: " << s << std::endl;
        s += v * dt;
        t += dt;
    }
}

double MotionModelDiffDrive::estimate_driving_time(const ControlParams& control)
{
    // acceleration time
    double t0 = fabs((control.vel.vt - control.vel.v0) / control.vel.a0);
    // deceleration time
    double td = fabs((control.vel.vf - control.vel.vt) / control.vel.af);
    // std::cout << t0 << ", " << td << std::endl;

    double s0 = 0.5 * fabs(control.vel.vt + control.vel.v0) * t0;
    double sd = 0.5 * fabs(control.vel.vt + control.vel.vf) * td;
    // std::cout << s0 << ", " << sd << std::endl;

    double st = fabs(control.omega.sf) - s0 - sd;
    double tt = st / fabs(control.vel.vt);
    // std::cout << st << ", " << tt << std::endl;
    double driving_time = t0 + tt + td;
    return driving_time;
}

