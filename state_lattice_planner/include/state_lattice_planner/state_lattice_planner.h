#ifndef __STATE_LATTICE_PLANNER_H
#define __STATE_LATTICE_PLANNER_H

#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

#include <trajectory_generator/motion_model_diff_drive.h>
#include <trajectory_generator/trajectory_generator_diff_drive.h>

class StateLatticePlanner
{
public:
    class SamplingParams
    {
    public:
        SamplingParams(void);
        SamplingParams(const int, const int, const double, const double);
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

    /// for lookup table
    class StateWithControlParams
    {
    public:
        StateWithControlParams(void);

        Eigen::Vector3d state;// x, y, yaw
        MotionModelDiffDrive::ControlParams control;
    private:
    };

    StateLatticePlanner(void);

    void process(void);
    void local_goal_callback(const geometry_msgs::PoseStampedConstPtr&);
    void local_map_callback(const nav_msgs::OccupancyGridConstPtr&);
    void odom_callback(const nav_msgs::OdometryConstPtr&);
    void generate_biased_polar_states(const int, const Eigen::Vector3d&, const SamplingParams&, std::vector<Eigen::Vector3d>&);
    void sample_states(const std::vector<double>&, const SamplingParams&, std::vector<Eigen::Vector3d>&);
    void generate_trajectories(const std::vector<Eigen::Vector3d>&, const double, const double, std::vector<MotionModelDiffDrive::Trajectory>&);
    bool check_collision(const nav_msgs::OccupancyGrid&, const std::vector<Eigen::Vector3d>&);
    bool pickup_trajectory(const std::vector<MotionModelDiffDrive::Trajectory>&, const Eigen::Vector3d&, MotionModelDiffDrive::Trajectory&);
    void load_lookup_table(void);
    void get_optimized_param_from_lookup_table(const Eigen::Vector3d, const double, const double, MotionModelDiffDrive::ControlParams&);

private:
    void swap(double&, double&);
    void generate_bresemhams_line(const std::vector<Eigen::Vector3d>&, const double&, std::vector<Eigen::Vector3d>&);
    void visualize_trajectories(const std::vector<MotionModelDiffDrive::Trajectory>&, const int, const int, const int, const ros::Publisher&);
    void visualize_trajectory(const MotionModelDiffDrive::Trajectory&, const int, const int, const int, const ros::Publisher&);

    double HZ;
    std::string ROBOT_FRAME;
    int N_P;
    int N_H;
    int N_S;
    double MAX_ALPHA;
    double MAX_PSI;
    double MAX_ACCELERATION;
    double TARGET_VELOCITY;
    std::string LOOKUP_TABLE_FILE_NAME;
    int MAX_ITERATION;
    double OPTIMIZATION_TOLERANCE;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Publisher velocity_pub;
    ros::Publisher candidate_trajectories_pub;
    ros::Publisher candidate_trajectories_no_collision_pub;
    ros::Publisher selected_trajectory_pub;
    ros::Subscriber local_map_sub;
    ros::Subscriber local_goal_sub;
    ros::Subscriber odom_sub;
    geometry_msgs::PoseStamped local_goal;
    nav_msgs::OccupancyGrid local_map;
    geometry_msgs::Twist current_velocity;
    bool local_goal_subscribed;
    bool local_map_updated;
    bool odom_updated;
    SamplingParams sampling_params;
    std::map<double, std::map<double, std::vector<StateWithControlParams> > > lookup_table;
};

#endif //__STATE_LATTICE_PLANNER_H
