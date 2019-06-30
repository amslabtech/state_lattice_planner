#include "trajectory_generator/trajectory_generator_diff_drive.h"

TrajectoryGeneratorDiffDrive::TrajectoryGeneratorDiffDrive(void)
{
    // default
    h << 0.005, 0.005, 0.1;
}

void TrajectoryGeneratorDiffDrive::set_optimization_param(const double dkm, const double dkf, const double dsf)
{
    h << dkm, dkf, dsf;
}

void TrajectoryGeneratorDiffDrive::set_motion_param(const double max_yawrate, const double max_curvature, const double max_d_curvature, const double max_acceleration)
{
    model.set_param(max_yawrate, max_curvature, max_d_curvature, max_acceleration);
}

double TrajectoryGeneratorDiffDrive::generate_optimized_trajectory(const Eigen::Vector3d& goal, const MotionModelDiffDrive::ControlParams& init_control_param, const double dt, const double tolerance, const int max_iteration, MotionModelDiffDrive::ControlParams& output, MotionModelDiffDrive::Trajectory& trajectory)
{
    Eigen::Vector3d cost(1e2, 1e2, 1e2);
    double last_cost = cost.norm();

    double distance_to_goal = goal.segment(0, 2).norm();

    output = init_control_param;

    int count = 0;

    Eigen::Matrix3d jacobian;

    while(1){
        //double start = ros::Time::now().toSec();
        if(cost.norm() < tolerance){
            //std::cout << "successfully optimized in " << count << " iteration" << std::endl;
            break;
        }else if(count >= max_iteration){
            std::cout << "cannot optimize trajectory" << std::endl;
            return -1;
        }
        trajectory.trajectory.clear();
        trajectory.velocities.clear();
        trajectory.angular_velocities.clear();
        double time = distance_to_goal / output.vel.v0;
        //std::cout << "bfr traj gen time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
        model.generate_trajectory(dt, output, trajectory);
        /*
        std::cout << "size: " << trajectory.trajectory.size() << std::endl;
        if(trajectory.trajectory.size() <= 1){
            std::cout << "failed to generate trajecotry!!!" << std::endl;
            return -1;
        }
        */
        //std::cout << "traj gen time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;

        get_jacobian(dt, output, h, jacobian);
        //std::cout << "get jacobian time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
        //std::cout << "j: \n" << jacobian << std::endl;
        //std::cout << "j^-1: \n" << jacobian.inverse() << std::endl;
        cost = goal - trajectory.trajectory.back();
        Eigen::Vector3d dp = jacobian.inverse() * cost;
        //Eigen::Vector3d dp = jacobian.lu().solve(cost);
        //std::cout << "jacobian inverse time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
        //std::cout << "cost: \n" << cost << std::endl;
        //std::cout << "dp: \n" << dp << std::endl;
        if((cost.norm() > last_cost) || std::isnan(dp(0)) || std::isnan(dp(1)) || std::isnan(dp(2)) || std::isinf(dp(0)) || std::isinf(dp(1)) || std::isinf(dp(2))){
            std::cout << "diverge to infinity!!!" << std::endl;
            return -1;
        }
        last_cost = cost.norm();

        output.curv.km += dp(0);
        output.curv.kf += dp(1);
        output.curv.sf += dp(2);

        if(fabs(output.curv.sf - distance_to_goal) > distance_to_goal * 0.5){
            std::cout << "optimization error!!!" << std::endl;
            return -1;
        }
        //std::cout << output.curv.km << ", " << output.curv.kf << ", " << output.curv.sf << std::endl; 

        //std::cout << "count: " << count << std::endl;
        count++;
        //std::cout << "optimization loop: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
    }
    //std::cout << "final cost: \n" << cost << std::endl;
    return cost.norm();
}

void TrajectoryGeneratorDiffDrive::get_jacobian(const double dt, const MotionModelDiffDrive::ControlParams& control, const Eigen::Vector3d& h, Eigen::Matrix3d& j)
{
    /*
     * h: (dkm, dkf, dsf)
     */
    //std::cout << "j start" << std::endl;
    double start = ros::Time::now().toSec();
    MotionModelDiffDrive::CurvatureParams curv = control.curv;
    Eigen::Vector3d x0;
    model.generate_last_state(dt, curv.sf, control.vel, curv.k0, curv.km - h(0), curv.kf, x0);
    Eigen::Vector3d x1;
    model.generate_last_state(dt, curv.sf, control.vel, curv.k0, curv.km + h(0), curv.kf, x1);

    Eigen::Vector3d dx_dkm;
    dx_dkm << (x1 - x0) / (2.0 * h(0));
    //std::cout << "dx_dkm time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;

    model.generate_last_state(dt, curv.sf, control.vel, curv.k0, curv.km, curv.kf - h(1), x0);
    model.generate_last_state(dt, curv.sf, control.vel, curv.k0, curv.km, curv.kf + h(1), x1);

    Eigen::Vector3d dx_dkf;
    dx_dkf << (x1 - x0) / (2.0 * h(1));
    //std::cout << "dx_dkf time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;

    model.generate_last_state(dt, curv.sf - h(2), control.vel, curv.k0, curv.km, curv.kf, x0);
    model.generate_last_state(dt, curv.sf + h(2), control.vel, curv.k0, curv.km, curv.kf, x1);

    Eigen::Vector3d dx_dsf;
    dx_dsf << (x1 - x0) / (2.0 * h(2));
    //std::cout << "dx_dsf time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;

    j << dx_dkm(0), dx_dkf(0), dx_dsf(0),
         dx_dkm(1), dx_dkf(1), dx_dsf(1),
         dx_dkm(2), dx_dkf(2), dx_dsf(2);
    //std::cout << "j time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
}
