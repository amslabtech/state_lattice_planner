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
        SamplingParams(const int, const int, const float, const float);
        SamplingParams(const int, const int, const float, const float, const float);

        int n_p;// num of sampling positions
        int n_h;// num of sampling angle by position
        float length;// sample trajectory length
        float max_alpha;// max trajectories angle
        float min_alpha;// min trajectories angle
        float span_alpha;// max - min alpha
        float max_psi;// max heading angle
        float min_psi;// min trajectories angle
        float span_psi;// max - min psi
    private:
    };

    /// for lookup table
    class StateWithControlParams
    {
    public:
        StateWithControlParams(void);

        Eigen::Vector3f state;// x, y, yaw
        MotionModelDiffDrive::ControlParams control;
    private:
    };

    StateLatticePlanner(void);

    void process(void);
    void local_goal_callback(const geometry_msgs::PoseStampedConstPtr&);
    void local_map_callback(const nav_msgs::OccupancyGridConstPtr&);
    void odom_callback(const nav_msgs::OdometryConstPtr&);
    void generate_biased_polar_states(const int, const Eigen::Vector3f&, const SamplingParams&, std::vector<Eigen::Vector3f>&);
    void sample_states(const std::vector<float>&, const SamplingParams&, std::vector<Eigen::Vector3f>&);
    bool generate_trajectories(const std::vector<Eigen::Vector3f>&, const float, const float, std::vector<MotionModelDiffDrive::Trajectory>&);
    bool check_collision(const nav_msgs::OccupancyGrid&, const std::vector<Eigen::Vector3f>&);
    bool pickup_trajectory(const std::vector<MotionModelDiffDrive::Trajectory>&, const Eigen::Vector3f&, MotionModelDiffDrive::Trajectory&);
    void load_lookup_table(void);
    void get_optimized_param_from_lookup_table(const Eigen::Vector3f, const float, const float, MotionModelDiffDrive::ControlParams&);

private:
    void swap(float&, float&);
    void generate_bresemhams_line(const std::vector<Eigen::Vector3f>&, const float&, std::vector<Eigen::Vector3f>&);
    void visualize_trajectories(const std::vector<MotionModelDiffDrive::Trajectory>&, const float, const float, const float, const int, const ros::Publisher&);
    void visualize_trajectory(const MotionModelDiffDrive::Trajectory&, const float, const float, const float, const ros::Publisher&);

    float HZ;
    std::string ROBOT_FRAME;
    int N_P;
    int N_H;
    int N_S;
    float MAX_ALPHA;
    float MAX_PSI;
    float MAX_ACCELERATION;
    float TARGET_VELOCITY;
    std::string LOOKUP_TABLE_FILE_NAME;
    int MAX_ITERATION;
    float OPTIMIZATION_TOLERANCE;
    float SHORTENING_TRAJECTORY_LENGTH_STEP;
    float SHORTENING_TRAJECTORY_MIN_LENGTH;
    float MAX_CURVATURE;
    float MAX_D_CURVATURE;
    float MAX_YAWRATE;

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
    std::map<float, std::map<float, std::vector<StateWithControlParams> > > lookup_table;
    float shortening_trajectory_length;
};

#endif //__STATE_LATTICE_PLANNER_H
