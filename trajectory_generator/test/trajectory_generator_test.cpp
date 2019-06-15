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
