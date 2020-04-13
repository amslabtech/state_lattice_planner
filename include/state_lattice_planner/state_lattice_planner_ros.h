#ifndef __STATE_LATTICE_PLANNER_ROS_H
#define __STATE_LATTICE_PLANNER_ROS_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "state_lattice_planner/state_lattice_planner.h"

class StateLatticePlannerROS
{
public:
    StateLatticePlannerROS(void);

    void process(void);
    void local_goal_callback(const geometry_msgs::PoseStampedConstPtr&);
    void local_map_callback(const nav_msgs::OccupancyGridConstPtr&);
    void odom_callback(const nav_msgs::OdometryConstPtr&);
    void target_velocity_callback(const geometry_msgs::TwistConstPtr&);
    template<typename TYPE>
    void get_obstacle_map(const nav_msgs::OccupancyGrid&, state_lattice_planner::ObstacleMap<TYPE>&);

protected:
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
    double IGNORABLE_OBSTACLE_RANGE;
    bool VERBOSE;
    int CONTROL_DELAY;
    double TURN_DIRECTION_THRESHOLD;
    bool ENABLE_SHARP_TRAJECTORY;
    bool ENABLE_CONTROL_SPACE_SAMPLING;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Publisher velocity_pub;
    ros::Publisher candidate_trajectories_pub;
    ros::Publisher candidate_trajectories_no_collision_pub;
    ros::Publisher selected_trajectory_pub;
    ros::Subscriber local_map_sub;
    ros::Subscriber local_goal_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber target_velocity_sub;
    tf::TransformListener listener;
    geometry_msgs::PoseStamped local_goal;
    nav_msgs::OccupancyGrid local_map;
    geometry_msgs::Twist current_velocity;
    bool local_goal_subscribed;
    bool local_map_updated;
    bool odom_updated;

    StateLatticePlanner planner;
};

#endif //__STATE_LATTICE_PLANNER_ROS_H
