#include "trajectory_generator/trajectory_generator_diff_drive.h"

TrajectoryGeneratorDiffDrive::TrajectoryGeneratorDiffDrive(void)
{

}

void TrajectoryGeneratorDiffDrive::get_jacobian(const Eigen::Vector3d& target, const double dt, const double v0, const double sf, const double k0, const double km, const double kf, const Eigen::Vector3d& h, Eigen::Matrix3d& j)
{
    /*
     * h: (dkm, dkf, dsf)
     */
    Eigen::Vector3d x0;
    model.generate_last_state(dt, sf, v0, k0, km - h(0), kf, x0);
    Eigen::Vector3d x1;
    model.generate_last_state(dt, sf, v0, k0, km + h(0), kf, x1);

    Eigen::Vector3d dp_dkm;
    dp_dkm << (x1(0) - x0(0)) / (2.0 * h(0)),
              (x1(1) - x0(1)) / (2.0 * h(0)),
              (x1(2) - x0(2)) / (2.0 * h(0));

    model.generate_last_state(dt, sf, v0, k0, km, kf - h(1), x0);
    model.generate_last_state(dt, sf, v0, k0, km, kf + h(1), x1);

    Eigen::Vector3d dp_dkf;
    dp_dkf << (x1(0) - x0(0)) / (2.0 * h(1)),
              (x1(1) - x0(1)) / (2.0 * h(1)),
              (x1(2) - x0(2)) / (2.0 * h(1));

    model.generate_last_state(dt, sf - h(2), v0, k0, km, kf, x0);
    model.generate_last_state(dt, sf + h(2), v0, k0, km, kf, x1);

    Eigen::Vector3d dp_dsf;
    dp_dsf << (x1(0) - x0(0)) / (2.0 * h(2)),
              (x1(1) - x0(1)) / (2.0 * h(2)),
              (x1(2) - x0(2)) / (2.0 * h(2));

    j << dp_dkm(0), dp_dkf(0), dp_dsf(0),
         dp_dkm(1), dp_dkf(1), dp_dsf(1),
         dp_dkm(2), dp_dkf(2), dp_dsf(2);
}
