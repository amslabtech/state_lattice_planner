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
    coeff_0_m = Eigen::Vector3d::Zero();
    coeff_m_f = Eigen::Vector3d::Zero();
}

MotionModelDiffDrive::CurvatureParams::CurvatureParams(double _k0, double _km, double _kf, double _sf)
{
    k0 = _k0;
    km = _km;
    kf = _kf;
    sf = _sf;
    coeff_0_m = Eigen::Vector3d::Zero();
    coeff_m_f = Eigen::Vector3d::Zero();
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

void MotionModelDiffDrive::generate_trajectory(const double dt, const ControlParams& control_param, Trajectory& trajectory)
{
    //double start = ros::Time::now().toSec();
    //std::cout << "gen start" << std::endl;
    CurvatureParams curv = control_param.curv;
    VelocityParams vel = control_param.vel;

    vel.time = estimate_driving_time(control_param);
    //std::cout << "estimate time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;

    make_velocity_profile(dt, vel);
    //std::cout << "v prof: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
    //std::cout << vel.v0 << ", " << vel.vt << ", " << vel.vf << ", " << vel.time << ", " << curv.sf << ", " << std::endl;
    //std::cout << "n: " << N << std::endl;

    curv.calculate_spline();
    //std::cout << "spline: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
    const int N = s_profile.size();
    std::vector<double> curv_profile;
    curv_profile.resize(N);
    double sf_2 = curv.sf * 0.5;
    int count = 0;
    for(const auto& s : s_profile){
        double c = 0;
        if(s < sf_2){
            c = calculate_quadratic_function(s, curv.coeff_0_m);
        }else{
            c = calculate_quadratic_function(s-sf_2, curv.coeff_m_f);
        }
        curv_profile[count] = c;
        count++;
    }
    //std::cout << "curv prof: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
    State state(0, 0, 0, vel.v0, curv.k0);
    State state_(0, 0, 0, vel.v0, curv.k0);
    Eigen::Vector3d pose;
    pose << state.x, state.y, state.yaw;
    trajectory.trajectory.resize(N);
    trajectory.velocities.resize(N);
    trajectory.angular_velocities.resize(N);
    trajectory.trajectory[0] = pose;
    trajectory.velocities[0] = state.v;
    trajectory.angular_velocities[0] = state.v * state.curvature;

    //start = ros::Time::now().toSec();
    for(int i=1;i<N;i++){
        //double u_start = ros::Time::now().toSec();
        update(state, v_profile[i], curv_profile[i], dt, state_);
        state = state_;
        pose << state.x, state.y, state.yaw;
        trajectory.trajectory[i] = pose;
        trajectory.velocities[i] = state.v;
        trajectory.angular_velocities[i] = state.v * state.curvature;
        //std::cout << "t" << i << ": " << ros::Time::now().toSec() - u_start << "[s]" << std::endl;
    }
    //std::cout << "gen t: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
}

void MotionModelDiffDrive::generate_last_state(const double dt, const double trajectory_length, const VelocityParams& _vel, const double k0, const double km, const double kf, Eigen::Vector3d& output)
{
    //std::cout << "--- generate last state ---" << std::endl;
    //double start = ros::Time::now().toSec();
    CurvatureParams curv(k0, km, kf, trajectory_length);
    VelocityParams vel = _vel;

    vel.time = estimate_driving_time(ControlParams(vel, curv));
    //std::cout << "estimate time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;

    make_velocity_profile(dt, vel);
    //std::cout << "v_profile time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;

    curv.calculate_spline();
    //std::cout << "spline time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
    const int N = s_profile.size();
    curv_profile.resize(N);
    double sf_2 = curv.sf * 0.5;
    int count = 0;
    for(const auto& s : s_profile){
        double c = 0;
        if(s < sf_2){
            c = calculate_quadratic_function(s, curv.coeff_0_m);
        }else{
            c = calculate_quadratic_function(s-sf_2, curv.coeff_m_f);
        }
        curv_profile[count] = c;
        count++;
    }
    //std::cout << "c_profile time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
    State state(0, 0, 0, vel.v0, curv.k0);
    output << state.x, state.y, state.yaw;

    for(int i=1;i<N;i++){
        update(state, v_profile[i], curv_profile[i], dt, state);
        output << state.x, state.y, state.yaw;
    }
    //std::cout << "finish time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
}

void MotionModelDiffDrive::CurvatureParams::calculate_spline(void)
{
    //std::cout << "spline" << std::endl;
    double start = ros::Time::now().toSec();
    // 2d spline interpolation
    Eigen::Vector3d x(0, sf * 0.5, sf);
    Eigen::Vector3d y(k0, km, kf);
    Eigen::Matrix3d s;
    //std::cout << "spline bfr s: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
    s << 2 * (x(1) - x(0)),             x(1), -x(1),
         x(1) * x(1),                   x(1), 0,
         (x(2) - x(1)) * (x(2) - x(1)), 0,    x(2) - x(1);
    Eigen::Vector3d c(0, y(1) - y(0), y(2) - y(1));
    //std::cout << "spline bfr inv: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
    Eigen::Vector3d a = s.inverse() * c;
    //std::cout << "spline aft inv: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
    coeff_0_m << a(0), a(1), y(0);
    coeff_m_f << a(0), a(2), y(1);
    //std::cout << "spline end: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
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

