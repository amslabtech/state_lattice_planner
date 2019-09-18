#include <gtest/gtest.h>

#include <ros/ros.h>

#include "state_lattice_planner/lookup_table_generator.h"
#include "state_lattice_planner/state_lattice_planner.h"

TEST(TestSuite, test0)
{
    LookupTableGenerator ltg;
    //ltg.process();
    EXPECT_NEAR(1.0, 1.0, 0.01);
}

TEST(TestSuite, test1)
{
    StateLatticePlanner slp;
    StateLatticePlanner::SamplingParams params(6, 3, 5.0, M_PI / 4.0, M_PI / 6.0);
    std::vector<double> angles = {0, 0.5, 1.0};
    std::vector<Eigen::Vector3d> states;
    slp.sample_states(angles, params, states);
    int n = 0;
    for(auto state : states){
        std::cout << "state " << n << std::endl;
        std::cout << state << std::endl;
        n++;
    }
    EXPECT_GT(n, 0);
}

TEST(TestSuite, test2)
{
    StateLatticePlanner slp;
    int np = 5;
    int nh = 2;
    int ns = 20;
    Eigen::Vector3d goal(2, 2, 0);
    StateLatticePlanner::SamplingParams params(np, nh, 5.0, M_PI / 4.0, M_PI / 6.0);
    std::vector<Eigen::Vector3d> states;
    double target_velocity = slp.get_target_velocity(goal);
    slp.generate_biased_polar_states(ns, goal, params, target_velocity, states);
    int n = 0;
    for(auto state : states){
        std::cout << "state " << n << std::endl;
        std::cout << state << std::endl;
        n++;
    }
    EXPECT_EQ(n, np * nh);
    //EXPECT_NEAR(states[0](2) - goal_direction, -states[n-1](2) - goal_direction, 1e-1);
}

TEST(TestSuite, test3)
{
    StateLatticePlanner slp;
    int np = 5;
    int nh = 2;
    int ns = 20;
    Eigen::Vector3d goal(5, 1, 1);
    StateLatticePlanner::SamplingParams params(np, nh, M_PI / 4.0, M_PI / 6.0);
    std::vector<Eigen::Vector3d> states;
    double target_velocity = slp.get_target_velocity(goal);
    slp.generate_biased_polar_states(ns, goal, params, target_velocity, states);
    int n = 0;
    for(auto state : states){
        std::cout << "state " << n << std::endl;
        std::cout << state << std::endl;
        n++;
    }
    std::vector<MotionModelDiffDrive::Trajectory> trajectories;
    slp.generate_trajectories(states, 0.5, 0, target_velocity, trajectories);
    int count = 0;
    for(auto trajectory : trajectories){
        std::cout << "trajectory " << count << std::endl;
        std::cout << trajectory.trajectory.back() << std::endl;
        count++;
    }
    Eigen::Vector3d center_state = trajectories[(np*nh)/2].trajectory.back();
    EXPECT_NEAR(center_state.segment(0, 2).norm(), goal.segment(0, 2).norm(), 0.1);
}

TEST(TestSuite, test4)
{
    StateLatticePlanner slp;
    int np = 10;
    int nh = 3;
    int ns = 1000;
    Eigen::Vector3d goal(5, -1, 1);
    StateLatticePlanner::SamplingParams params(np, nh, M_PI / 4.0, M_PI / 6.0);
    double start = ros::Time::now().toSec();
    std::vector<Eigen::Vector3d> states;
    double target_velocity = slp.get_target_velocity(goal);
    slp.generate_biased_polar_states(ns, goal, params, target_velocity, states);
    std::vector<MotionModelDiffDrive::Trajectory> trajectories;
    slp.generate_trajectories(states, 0.0, 0.0, target_velocity, trajectories);
    MotionModelDiffDrive::Trajectory trajectory;
    slp.pickup_trajectory(trajectories, goal, trajectory);
    std::cout << "time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
    std::cout << "goal" << std::endl;
    std::cout << goal << std::endl;
    std::cout << "terminal state" << std::endl;
    std::cout << trajectory.trajectory.back() << std::endl;
    std::cout << "velocity" << std::endl;
    std::cout << trajectory.velocities.back() << std::endl;
    int size = trajectory.trajectory.size();
    for(int i=0;i<size;i++){
        std::cout << trajectory.velocities[i] << "[m/s]" << trajectory.angular_velocities[i] << "[rad/s]" << std::endl;
    }
    EXPECT_LT((goal.segment(0, 2) - trajectory.trajectory.back().segment(0, 2)).norm(), 0.2);
}

TEST(TestSuite, test5)
{
    StateLatticePlanner slp;
    slp.load_lookup_table();
    Eigen::Vector3d goal(3, 1, 0.5);
    double v0 = 0.5;
    double k0 = -0.1;
    MotionModelDiffDrive::ControlParams control;
    slp.get_optimized_param_from_lookup_table(goal, v0, k0, control);
    std::cout << control.omega.k0 << ", " << control.omega.km << ", " << control.omega.kf << ", " << control.omega.sf << std::endl;
}

TEST(TestSuite, test6)
{
    StateLatticePlanner slp;
    int np = 10;
    int nh = 3;
    int ns = 1000;
    Eigen::Vector3d goal(-5, 1, -0.5);
    StateLatticePlanner::SamplingParams params(np, nh, M_PI / 4.0, M_PI / 6.0);
    double start = ros::Time::now().toSec();
    std::vector<Eigen::Vector3d> states;
    double target_velocity = slp.get_target_velocity(goal);
    slp.generate_biased_polar_states(ns, goal, params, target_velocity, states);
    for(const auto& s : states){
        std::cout << s.transpose() << std::endl;
    }
    std::vector<MotionModelDiffDrive::Trajectory> trajectories;
    slp.generate_trajectories(states, 0.0, 0.0, target_velocity, trajectories);
    MotionModelDiffDrive::Trajectory trajectory;
    slp.pickup_trajectory(trajectories, goal, trajectory);
    std::cout << "time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
    std::cout << "goal" << std::endl;
    std::cout << goal << std::endl;
    std::cout << "terminal state" << std::endl;
    std::cout << trajectory.trajectory.back() << std::endl;
    std::cout << "velocity" << std::endl;
    std::cout << trajectory.velocities.back() << std::endl;
    int size = trajectory.trajectory.size();
    for(int i=0;i<size;i++){
        std::cout << trajectory.velocities[i] << "[m/s]" << trajectory.angular_velocities[i] << "[rad/s]" << std::endl;
    }
    EXPECT_LT((goal.segment(0, 2) - trajectory.trajectory.back().segment(0, 2)).norm(), 0.2);
}
int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "state_lattice_planner_test");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Duration(3.0).sleep();

    int r_e_t = RUN_ALL_TESTS();

    spinner.stop();

    ros::shutdown();

    return r_e_t;
}
