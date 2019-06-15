#include <gtest/gtest.h>

#include <ros/ros.h>


#include "trajectory_generator/motion_model_diff_drive.h"
#include "trajectory_generator/trajectory_generator_diff_drive.h"

TEST(TestSuite, test0)
{
	ros::NodeHandle nh;
    MotionModelDiffDrive::CurvatureParams curv(0, 0.5, 1.0, 5);
    curv.calculate_spline();
    MotionModelDiffDrive mm;
    double cf = mm.calculate_cubic_function(0, curv.coeff_0_m);
	EXPECT_NEAR(0, cf, 0.01);
    cf = mm.calculate_cubic_function(2.5, curv.coeff_0_m);
	EXPECT_NEAR(0.5, cf, 0.01);
    cf = mm.calculate_cubic_function(2.5, curv.coeff_m_f);
	EXPECT_NEAR(0.5, cf, 0.01);
    cf = mm.calculate_cubic_function(5, curv.coeff_m_f);
	EXPECT_NEAR(1.0, cf, 0.01);
}

TEST(TestSuite, test1)
{
	ros::NodeHandle nh;
    MotionModelDiffDrive::CurvatureParams curv(0, -0.5, -1.0, 10);
    curv.calculate_spline();
    MotionModelDiffDrive mm;
    double cf = mm.calculate_cubic_function(0, curv.coeff_0_m);
	EXPECT_NEAR(0, cf, 0.01);
    cf = mm.calculate_cubic_function(5, curv.coeff_0_m);
	EXPECT_NEAR(-0.5, cf, 0.01);
    cf = mm.calculate_cubic_function(5, curv.coeff_m_f);
	EXPECT_NEAR(-0.5, cf, 0.01);
    cf = mm.calculate_cubic_function(10, curv.coeff_m_f);
	EXPECT_NEAR(-1.0, cf, 0.01);
}

TEST(TestSuite, test2)
{
	ros::NodeHandle nh;
    MotionModelDiffDrive mm;
    MotionModelDiffDrive::CurvatureParams curv(0.0, 0.0, 0.0, 5);
    std::vector<Eigen::Vector3d> trajectory;
    mm.generate_trajectory(0.01, 0.5, curv, trajectory);

    EXPECT_NEAR(5, trajectory.back()(0), 0.05);
    EXPECT_NEAR(0, trajectory.back()(1), 0.05);
    EXPECT_NEAR(0, trajectory.back()(2), 0.05);
}

TEST(TestSuite, test3)
{
	ros::NodeHandle nh;
    TrajectoryGeneratorDiffDrive tg;
    MotionModelDiffDrive::CurvatureParams output_c;
    MotionModelDiffDrive::VelocityParams output_v;
    MotionModelDiffDrive::CurvatureParams curv(0.0, 0.0, 0.5, 5);
    MotionModelDiffDrive::VelocityParams vel(0.5, 0.0);
    Eigen::Vector3d goal(5, 1, 1);
    std::vector<Eigen::Vector3d> trajectory;
    double cost = tg.generate_optimized_trajectory(goal, vel, curv, 0.01, 1e-3, 1000, output_v, output_c, trajectory);
    std::cout << "trajecotry.back():" << std::endl;
    std::cout << trajectory.back() << std::endl;
    std::cout << "cost: " << cost << std::endl;

    EXPECT_NEAR(5, trajectory.back()(0), 0.01);
    EXPECT_NEAR(1, trajectory.back()(1), 0.01);
    EXPECT_NEAR(1, trajectory.back()(2), 0.01);
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);

	ros::init(argc, argv, "trajectory_generator_test");

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Duration(3.0).sleep();

	int r_e_t = RUN_ALL_TESTS();

	spinner.stop();

	ros::shutdown();

	return r_e_t;
}
