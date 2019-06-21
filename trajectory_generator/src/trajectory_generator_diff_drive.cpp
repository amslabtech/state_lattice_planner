#include "trajectory_generator/trajectory_generator_diff_drive.h"

TrajectoryGeneratorDiffDrive::TrajectoryGeneratorDiffDrive(void)
{
    // default
    h << 0.005, 0.005, 0.1;
}

void TrajectoryGeneratorDiffDrive::set_param(const double dkm, const double dkf, const double dsf)
{
    h << dkm, dkf, dsf;
}

double TrajectoryGeneratorDiffDrive::generate_optimized_trajectory(const Eigen::Vector3d& goal, const MotionModelDiffDrive::ControlParams& init_control_param, const double dt, const double tolerance, const int max_iteration, MotionModelDiffDrive::ControlParams& output, std::vector<Eigen::Vector3d>& trajectory)
{
    Eigen::Vector3d cost(1, 1, 1);

    output = init_control_param;

    int count = 0;

    while(1){
        if(cost.norm() < tolerance){
            break;
        }else if(count >= max_iteration){
            std::cout << "cannot optimize trajectory" << std::endl;
            return -1;
        }
        trajectory.clear();
        double time = goal.norm() / output.vel.v0;
        model.generate_trajectory(dt, output.vel.v0, output.curv, trajectory);

        Eigen::Matrix3d jacobian;
        get_jacobian(dt, output.vel.v0, output.curv, h, jacobian);
        //std::cout << "j: \n" << jacobian << std::endl;
        //std::cout << "j^-1: \n" << jacobian.inverse() << std::endl;
        cost = goal - trajectory.back();
        Eigen::Vector3d dp = jacobian.inverse() * cost;
        //std::cout << "cost: \n" << cost << std::endl;
        //std::cout << "dp: \n" << dp << std::endl;
        if(dp.norm() > 1e2){
            std::cout << "diverge to infinity!!!" << std::endl;
            return -1;
        }

        output.curv.km += dp(0);
        output.curv.kf += dp(1);
        output.curv.sf += dp(2);

        //std::cout << "count: " << count << std::endl;
        count++;
    }
    return cost.norm();
}

void TrajectoryGeneratorDiffDrive::get_jacobian(const double dt, const double v0, const MotionModelDiffDrive::CurvatureParams& curv, const Eigen::Vector3d& h, Eigen::Matrix3d& j)
{
    /*
     * h: (dkm, dkf, dsf)
     */
    Eigen::Vector3d x0;
    model.generate_last_state(dt, curv.sf, v0, curv.k0, curv.km - h(0), curv.kf, x0);
    Eigen::Vector3d x1;
    model.generate_last_state(dt, curv.sf, v0, curv.k0, curv.km + h(0), curv.kf, x1);

    Eigen::Vector3d dx_dkm;
    dx_dkm << (x1 - x0) / (2.0 * h(0));

    model.generate_last_state(dt, curv.sf, v0, curv.k0, curv.km, curv.kf - h(1), x0);
    model.generate_last_state(dt, curv.sf, v0, curv.k0, curv.km, curv.kf + h(1), x1);

    Eigen::Vector3d dx_dkf;
    dx_dkf << (x1 - x0) / (2.0 * h(1));

    model.generate_last_state(dt, curv.sf - h(2), v0, curv.k0, curv.km, curv.kf, x0);
    model.generate_last_state(dt, curv.sf + h(2), v0, curv.k0, curv.km, curv.kf, x1);

    Eigen::Vector3d dx_dsf;
    dx_dsf << (x1 - x0) / (2.0 * h(2));

    j << dx_dkm(0), dx_dkf(0), dx_dsf(0),
         dx_dkm(1), dx_dkf(1), dx_dsf(1),
         dx_dkm(2), dx_dkf(2), dx_dsf(2);
}
