#ifndef __TRAJECTORY_VIEWER_H
#define __TRAJECTORY_VIEWER_H

#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>

#include <omp.h>

#include <trajectory_generator/motion_model_diff_drive.h>
#include <trajectory_generator/trajectory_generator_diff_drive.h>
#include "state_lattice_planner/lookup_table_utils.h"

class TrajectoryViewer
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

    TrajectoryViewer(void);

    void process(void);
    void local_goal_callback(const geometry_msgs::PoseStampedConstPtr&);
    void local_map_callback(const nav_msgs::OccupancyGridConstPtr&);
    void target_velocity_callback(const geometry_msgs::TwistConstPtr&);
    void generate_biased_polar_states(const int, const Eigen::Vector3d&, const SamplingParams&, double, std::vector<Eigen::Vector3d>&);
    void sample_states(const std::vector<double>&, const SamplingParams&, std::vector<Eigen::Vector3d>&);
    bool generate_trajectories(const std::vector<Eigen::Vector3d>&, const double, const double, const double, std::vector<MotionModelDiffDrive::Trajectory>&);
    bool pickup_trajectory(const std::vector<MotionModelDiffDrive::Trajectory>&, const Eigen::Vector3d&, MotionModelDiffDrive::Trajectory&);
    void load_lookup_table(void);
    void get_optimized_param_from_lookup_table(const Eigen::Vector3d, const double, const double, MotionModelDiffDrive::ControlParams&);
    double get_target_velocity(const Eigen::Vector3d&);

protected:
    void swap(double&, double&);
    void visualize_trajectories(const std::vector<MotionModelDiffDrive::Trajectory>&, const double, const double, const double, const int, const ros::Publisher&);
    void visualize_trajectory(const MotionModelDiffDrive::Trajectory&, const double, const double, const double, const ros::Publisher&);

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
    double MAX_YAWRATE;
    double MAX_D_YAWRATE;
    double MAX_WHEEL_ANGULAR_VELOCITY;
    double WHEEL_RADIUS;
    double TREAD;
    bool VERBOSE;
    double V0;
    double K0;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Publisher candidate_trajectories_pub;
    ros::Publisher selected_trajectory_pub;
    ros::Subscriber local_goal_sub;
    ros::Subscriber target_velocity_sub;
    geometry_msgs::PoseStamped local_goal;
    geometry_msgs::Twist current_velocity;
    bool local_goal_updated;
    SamplingParams sampling_params;
    LookupTableUtils::LookupTable lookup_table;
};

#endif //__TRAJECTORY_VIEWER_H
