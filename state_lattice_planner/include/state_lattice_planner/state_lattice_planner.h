#ifndef __STATE_LATTICE_PLANNER_H
#define __STATE_LATTICE_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>

#include <trajectory_generator/motion_model_diff_drive.h>
#include <trajectory_generator/trajectory_generator_diff_drive.h>

class StateLatticePlanner
{
public:
    class SamplingParams
    {
    public:
        SamplingParams(const int, const int, const double, const double, const double);

        int n_p;// num of sampling positions
        int n_h;// num of sampling angle by position
        double length;// sample trajectory length
        double max_alpha;// max trajectories angle
        double min_alpha;// min trajectories angle
        double span_alpha;// max - min alpha
        double max_psi;// max heading angle
        double min_psi;// min trajectories angle
        double span_psi;// max - min psi
    private:
    };

    StateLatticePlanner(void);

    void process(void);
    void local_goal_callback(const geometry_msgs::PoseStampedConstPtr&);
    void local_map_callback(const nav_msgs::OccupancyGridConstPtr&);
    void generate_biased_polar_states(const int, const Eigen::Vector3d&, const SamplingParams&, std::vector<Eigen::Vector3d>&);
    void sample_states(const std::vector<double>&, const SamplingParams&, std::vector<Eigen::Vector3d>&);


private:
    double HZ;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Publisher velocity_pub;
    ros::Subscriber local_map_sub;
    ros::Subscriber local_goal_sub;
    geometry_msgs::PoseStamped local_goal;
    nav_msgs::OccupancyGrid local_map;
    bool local_goal_subscribed;
    bool local_map_updated;
};

#endif //__STATE_LATTICE_PLANNER_H
