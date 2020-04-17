#include <gtest/gtest.h>

#include "state_lattice_planner/lookup_table_utils.h"
#include "state_lattice_planner/state_lattice_planner.h"

TEST(StateLatticePlannerFunctionTest, SamplingState)
{
    StateLatticePlanner slp;
    StateLatticePlanner::SamplingParams params(6, 3, 5.0, M_PI / 4.0, M_PI / 6.0);
    std::vector<double> angles = {0, 0.5, 1.0};
    std::vector<Eigen::Vector3d> states;
    slp.set_sampling_params(params);
    slp.sample_states(angles, states);
    int n = 0;
    for(auto state : states){
        std::cout << "state " << n << std::endl;
        std::cout << state << std::endl;
        n++;
    }
    EXPECT_GT(n, 0);
}

TEST(StateLatticePlannerFunctionTest, GenerateTerminalStates)
{
    StateLatticePlanner slp;
    int np = 5;
    int nh = 2;
    int ns = 20;
    Eigen::Vector3d goal(2, 2, 0);
    StateLatticePlanner::SamplingParams params(np, nh, 5.0, M_PI / 4.0, M_PI / 6.0);
    slp.set_sampling_params(params);
    std::vector<Eigen::Vector3d> states;
    double target_velocity = slp.get_target_velocity(goal);
    slp.generate_biased_polar_states(ns, goal, target_velocity, states);
    int n = 0;
    for(auto state : states){
        std::cout << "state " << n << std::endl;
        std::cout << state << std::endl;
        n++;
    }
    EXPECT_EQ(n, np * nh);
    //EXPECT_NEAR(states[0](2) - goal_direction, -states[n-1](2) - goal_direction, 1e-1);
}

TEST(StateLatticePlannerFunctionTest, CompareCenterWithGoal)
{
    StateLatticePlanner slp;
    int np = 5;
    int nh = 2;
    int ns = 20;
    Eigen::Vector3d goal(5, 1, 1);
    StateLatticePlanner::SamplingParams params(np, nh, M_PI / 4.0, M_PI / 6.0);
    slp.set_sampling_params(params);
    std::vector<Eigen::Vector3d> states;
    double target_velocity = slp.get_target_velocity(goal);
    slp.generate_biased_polar_states(ns, goal, target_velocity, states);
    int n = 0;
    for(auto state : states){
        std::cout << "state " << n << std::endl;
        std::cout << state << std::endl;
        n++;
    }
    std::vector<MotionModelDiffDrive::Trajectory> trajectories;
    slp.generate_trajectories(states, 0.5, 0, target_velocity, trajectories);
    std::cout << "traj: " << trajectories.size() << std::endl;
    int count = 0;
    for(auto trajectory : trajectories){
        std::cout << "trajectory " << count << std::endl;
        std::cout << trajectory.trajectory.back() << std::endl;
        count++;
    }
    Eigen::Vector3d center_state = trajectories[(np*nh)/2].trajectory.back();
    EXPECT_NEAR(center_state.segment(0, 2).norm(), goal.segment(0, 2).norm(), 0.1);
}

TEST(StateLatticePlannerFunctionTest, GenerateTrajectories)
{
    StateLatticePlanner slp;
    int np = 10;
    int nh = 3;
    int ns = 1000;
    Eigen::Vector3d goal(5, -1, 1);
    StateLatticePlanner::SamplingParams params(np, nh, M_PI / 4.0, M_PI / 6.0);
    slp.set_sampling_params(params);
    std::vector<Eigen::Vector3d> states;
    double target_velocity = slp.get_target_velocity(goal);
    slp.generate_biased_polar_states(ns, goal, target_velocity, states);
    std::vector<MotionModelDiffDrive::Trajectory> trajectories;
    slp.generate_trajectories(states, 0.0, 0.0, target_velocity, trajectories);
    MotionModelDiffDrive::Trajectory trajectory;
    slp.pickup_trajectory(trajectories, goal, trajectory);
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

TEST(StateLatticePlannerFunctionTest, GenerateBackTrajectories)
{
    StateLatticePlanner slp;
    int np = 10;
    int nh = 3;
    int ns = 1000;
    Eigen::Vector3d goal(-5, 1, -0.5);
    StateLatticePlanner::SamplingParams params(np, nh, M_PI / 4.0, M_PI / 6.0);
    slp.set_sampling_params(params);
    std::vector<Eigen::Vector3d> states;
    double target_velocity = slp.get_target_velocity(goal);
    slp.generate_biased_polar_states(ns, goal, target_velocity, states);
    for(const auto& s : states){
        std::cout << s.transpose() << std::endl;
    }
    std::vector<MotionModelDiffDrive::Trajectory> trajectories;
    slp.generate_trajectories(states, 0.0, 0.0, target_velocity, trajectories);
    MotionModelDiffDrive::Trajectory trajectory;
    slp.pickup_trajectory(trajectories, goal, trajectory);
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
    int r_e_t = RUN_ALL_TESTS();
    return r_e_t;
}
