#include "state_lattice_planner/state_lattice_planner_ros.h"

StateLatticePlannerROS::StateLatticePlannerROS(void)
:local_nh("~")
{
    local_nh.param("HZ", HZ, {20});
    local_nh.param("ROBOT_FRAME", ROBOT_FRAME, {"base_link"});
    local_nh.param("N_P", N_P, {10});
    local_nh.param("N_H", N_H, {3});
    local_nh.param("MAX_ALPHA", MAX_ALPHA, {M_PI / 3.0});
    local_nh.param("MAX_PSI", MAX_PSI, {M_PI / 6.0});
    local_nh.param("N_S", N_S, {1000});
    local_nh.param("MAX_ACCELERATION", MAX_ACCELERATION, {1.0});
    local_nh.param("TARGET_VELOCITY", TARGET_VELOCITY, {0.8});
    local_nh.param("LOOKUP_TABLE_FILE_NAME", LOOKUP_TABLE_FILE_NAME, {std::string(std::getenv("HOME")) + "/lookup_table.csv"});
    local_nh.param("MAX_ITERATION", MAX_ITERATION, {100});
    local_nh.param("OPTIMIZATION_TOLERANCE", OPTIMIZATION_TOLERANCE, {0.1});
    local_nh.param("MAX_YAWRATE", MAX_YAWRATE, {0.8});
    local_nh.param("MAX_D_YAWRATE", MAX_D_YAWRATE, {2.0});
    local_nh.param("MAX_WHEEL_ANGULAR_VELOCITY", MAX_WHEEL_ANGULAR_VELOCITY, {11.6});
    local_nh.param("WHEEL_RADIUS", WHEEL_RADIUS, {0.125});
    local_nh.param("TREAD", TREAD, {0.5});
    local_nh.param("IGNORABLE_OBSTACLE_RANGE", IGNORABLE_OBSTACLE_RANGE, {1.0});
    local_nh.param("VERBOSE", VERBOSE, {false});
    local_nh.param("CONTROL_DELAY", CONTROL_DELAY, {1});
    local_nh.param("TURN_DIRECTION_THRESHOLD", TURN_DIRECTION_THRESHOLD, {M_PI/4.0});
    local_nh.param("ENABLE_SHARP_TRAJECTORY", ENABLE_SHARP_TRAJECTORY, {false});
    local_nh.param("ENABLE_CONTROL_SPACE_SAMPLING", ENABLE_CONTROL_SPACE_SAMPLING, {false});

    std::cout << "HZ: " << HZ << std::endl;
    std::cout << "ROBOT_FRAME: " << ROBOT_FRAME << std::endl;
    std::cout << "N_P: " << N_P << std::endl;
    std::cout << "N_H: " << N_H << std::endl;
    std::cout << "MAX_ALPHA: " << MAX_ALPHA << std::endl;
    std::cout << "MAX_PSI: " << MAX_PSI << std::endl;
    std::cout << "N_S: " << N_S << std::endl;
    std::cout << "MAX_ACCELERATION: " << MAX_ACCELERATION << std::endl;
    std::cout << "TARGET_VELOCITY: " << TARGET_VELOCITY << std::endl;
    std::cout << "LOOKUP_TABLE_FILE_NAME: " << LOOKUP_TABLE_FILE_NAME << std::endl;
    std::cout << "MAX_ITERATION: " << MAX_ITERATION << std::endl;
    std::cout << "OPTIMIZATION_TOLERANCE: " << OPTIMIZATION_TOLERANCE << std::endl;
    std::cout << "MAX_YAWRATE: " << MAX_YAWRATE << std::endl;
    std::cout << "MAX_D_YAWRATE: " << MAX_D_YAWRATE << std::endl;
    std::cout << "MAX_WHEEL_ANGULAR_VELOCITY: " << MAX_WHEEL_ANGULAR_VELOCITY << std::endl;
    std::cout << "WHEEL_RADIUS: " << WHEEL_RADIUS << std::endl;
    std::cout << "TREAD: " << TREAD << std::endl;
    std::cout << "IGNORABLE_OBSTACLE_RANGE: " << IGNORABLE_OBSTACLE_RANGE << std::endl;
    std::cout << "VERBOSE: " << VERBOSE << std::endl;
    std::cout << "CONTROL_DELAY: " << CONTROL_DELAY << std::endl;
    std::cout << "TURN_DIRECTION_THRESHOLD: " << TURN_DIRECTION_THRESHOLD << std::endl;
    std::cout << "ENABLE_SHARP_TRAJECTORY: " << ENABLE_SHARP_TRAJECTORY << std::endl;
    std::cout << "ENABLE_CONTROL_SPACE_SAMPLING: " << ENABLE_CONTROL_SPACE_SAMPLING << std::endl;

    planner.set_sampling_params(StateLatticePlanner::SamplingParams(N_P, N_H, MAX_ALPHA, MAX_PSI));
    planner.set_optimization_params(MAX_ITERATION, OPTIMIZATION_TOLERANCE);
    planner.set_vehicle_params(WHEEL_RADIUS, TREAD);
    planner.set_motion_params(MAX_ACCELERATION, MAX_YAWRATE, MAX_D_YAWRATE);
    planner.set_target_velocity(TARGET_VELOCITY);

    velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    candidate_trajectories_pub = local_nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
    candidate_trajectories_no_collision_pub = local_nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories/no_collision", 1);
    selected_trajectory_pub = local_nh.advertise<visualization_msgs::Marker>("selected_trajectory", 1);

    local_goal_sub = nh.subscribe("/local_goal", 1, &StateLatticePlannerROS::local_goal_callback, this);
    local_map_sub = nh.subscribe("/local_map", 1, &StateLatticePlannerROS::local_map_callback, this);
    odom_sub = nh.subscribe("/odom", 1, &StateLatticePlannerROS::odom_callback, this);
    target_velocity_sub = nh.subscribe("/target_velocity", 1, &StateLatticePlannerROS::target_velocity_callback, this);

    local_goal_subscribed = false;
    local_map_updated = false;
    odom_updated = false;

    planner.load_lookup_table(LOOKUP_TABLE_FILE_NAME);
}

void StateLatticePlannerROS::local_goal_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    local_goal = *msg;
    try{
        listener.transformPose("/odom", ros::Time(0), local_goal, local_goal.header.frame_id, local_goal);
        local_goal_subscribed = true;
    }catch(tf::TransformException ex){
        std::cout << ex.what() << std::endl;
    }
}

void StateLatticePlannerROS::local_map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    local_map = *msg;
    local_map_updated = true;
}

void StateLatticePlannerROS::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    current_velocity = msg->twist.twist;
    odom_updated = true;
}

void StateLatticePlannerROS::target_velocity_callback(const geometry_msgs::TwistConstPtr& msg)
{
    if(msg->linear.x > 0.0){
        TARGET_VELOCITY = msg->linear.x;
        std::cout << "\033[31mtarget velocity was updated to " << TARGET_VELOCITY << "[m/s]\033[0m" << std::endl;
    }
}

void StateLatticePlannerROS::process(void)
{
    ros::Rate loop_rate(HZ);

    while(ros::ok()){
        bool goal_transformed = false;
        geometry_msgs::PoseStamped local_goal_base_link;
        if(local_goal_subscribed){
            try{
                listener.transformPose(ROBOT_FRAME, ros::Time(0), local_goal, local_goal.header.frame_id, local_goal_base_link);
                goal_transformed = true;
            }catch(tf::TransformException ex){
                std::cout << ex.what() << std::endl;
            }
        }
        if(local_goal_subscribed && local_map_updated && odom_updated && goal_transformed){
            std::cout << "=== state lattice planner ===" << std::endl;
            double start = ros::Time::now().toSec();
            static int last_trajectory_num = 0;
            std::cout << "local goal: \n" << local_goal_base_link << std::endl;
            std::cout << "current_velocity: \n" << current_velocity << std::endl;
            Eigen::Vector3d goal(local_goal_base_link.pose.position.x, local_goal_base_link.pose.position.y, tf::getYaw(local_goal_base_link.pose.orientation));
            std::vector<Eigen::Vector3d> states;
            double target_velocity = planner.get_target_velocity(goal);
            planner.generate_biased_polar_states(N_S, goal, target_velocity, states);
            std::vector<MotionModelDiffDrive::Trajectory> trajectories;
            bool generated = planner.generate_trajectories(states, current_velocity.linear.x, current_velocity.angular.z, target_velocity, trajectories);
            if(ENABLE_SHARP_TRAJECTORY){
                generated |= planner.generate_trajectories(states, current_velocity.linear.x, current_velocity.angular.z + MAX_D_YAWRATE / HZ, target_velocity, trajectories);
                generated |= planner.generate_trajectories(states, current_velocity.linear.x, current_velocity.angular.z - MAX_D_YAWRATE / HZ, target_velocity, trajectories);
            }
            bool turn_flag = false;
            double relative_direction = atan2(local_goal_base_link.pose.position.y, local_goal_base_link.pose.position.x);
            if(goal.segment(0, 2).norm() < 0.1){
                generated = false;
            }else if(fabs(relative_direction) > TURN_DIRECTION_THRESHOLD){
                if(fabs(goal(2)) > TURN_DIRECTION_THRESHOLD){
                    generated = false;
                    turn_flag = true;
                }
            }
            if(generated){
                visualize_trajectories(trajectories, 0, 1, 0, last_trajectory_num, candidate_trajectories_pub);

                std::cout << "check candidate trajectories" << std::endl;
                std::vector<MotionModelDiffDrive::Trajectory> candidate_trajectories;
                state_lattice_planner::ObstacleMap<int> obstacle_map;
                get_obstacle_map(local_map, obstacle_map);
                for(const auto& trajectory : trajectories){
                    if(!planner.check_collision(obstacle_map, trajectory.trajectory)){
                        candidate_trajectories.push_back(trajectory);
                    }
                }
                std::cout << "trajectories: " << trajectories.size() << std::endl;
                std::cout << "candidate_trajectories: " << candidate_trajectories.size() << std::endl;
                if(candidate_trajectories.empty()){
                    // if no candidate trajectories
                    // collision checking with relaxed restrictions
                    for(const auto& trajectory : trajectories){
                        if(!planner.check_collision(obstacle_map, trajectory.trajectory, IGNORABLE_OBSTACLE_RANGE)){
                            candidate_trajectories.push_back(trajectory);
                        }
                    }
                    std::cout << "candidate_trajectories(ignore far obstacles): " << candidate_trajectories.size() << std::endl;
                }
                // std::cout << "candidate time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
                if(candidate_trajectories.size() > 0){
                    visualize_trajectories(candidate_trajectories, 0, 0.5, 1, last_trajectory_num, candidate_trajectories_no_collision_pub);

                    std::cout << "pickup a optimal trajectory from candidate trajectories" << std::endl;
                    MotionModelDiffDrive::Trajectory trajectory;
                    planner.pickup_trajectory(candidate_trajectories, goal, trajectory);
                    visualize_trajectory(trajectory, 1, 0, 0, selected_trajectory_pub);
                    std::cout << "pickup time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;

                    // int size = trajectory.trajectory.size();
                    // for(int i=0;i<size;i++){
                    //     std::cout << trajectory.trajectory[i].transpose() << ", " << trajectory.velocities[i] << "[m/s], " << trajectory.angular_velocities[i] << "[rad/s]" << std::endl;
                    // }

                    std::cout << "publish velocity" << std::endl;
                    geometry_msgs::Twist cmd_vel;
                    double calculation_time = ros::Time::now().toSec() - start;
                    int delayed_control_index = std::min(std::ceil(calculation_time * HZ) + CONTROL_DELAY, (double)trajectory.trajectory.size());
                    if((int)trajectory.trajectory.size() < CONTROL_DELAY){
                        delayed_control_index = std::ceil(calculation_time * HZ);
                    }
                    std::cout << calculation_time << ", " << delayed_control_index << std::endl;
                    cmd_vel.linear.x = trajectory.velocities[delayed_control_index];
                    cmd_vel.angular.z = trajectory.angular_velocities[delayed_control_index];
                    velocity_pub.publish(cmd_vel);
                    std::cout << "published velocity: \n" << cmd_vel << std::endl;

                    local_map_updated = false;
                    odom_updated = false;
                }else{
                    std::cout << "\033[91mERROR: stacking\033[00m" << std::endl;
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    velocity_pub.publish(cmd_vel);
                    // for clear
                    std::vector<MotionModelDiffDrive::Trajectory> clear_trajectories;
                    visualize_trajectories(clear_trajectories, 0, 1, 0, last_trajectory_num, candidate_trajectories_pub);
                    visualize_trajectories(clear_trajectories, 0, 0.5, 1, last_trajectory_num, candidate_trajectories_no_collision_pub);
                    visualize_trajectory(MotionModelDiffDrive::Trajectory(), 1, 0, 0, selected_trajectory_pub);
                }
            }else{
                std::cout << "\033[91mERROR: no optimized trajectory was generated\033[00m" << std::endl;
                std::cout << "\033[91mturn for local goal\033[00m" << std::endl;
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = 0;
                if(!turn_flag){
                    cmd_vel.angular.z = std::min(std::max(goal(2), -MAX_YAWRATE), MAX_YAWRATE);
                }else{
                    cmd_vel.angular.z = std::min(std::max(relative_direction, -MAX_YAWRATE), MAX_YAWRATE);
                }
                velocity_pub.publish(cmd_vel);
                // for clear
                std::vector<MotionModelDiffDrive::Trajectory> clear_trajectories;
                visualize_trajectories(clear_trajectories, 0, 1, 0, last_trajectory_num, candidate_trajectories_pub);
                visualize_trajectories(clear_trajectories, 0, 0.5, 1, last_trajectory_num, candidate_trajectories_no_collision_pub);
                visualize_trajectory(MotionModelDiffDrive::Trajectory(), 1, 0, 0, selected_trajectory_pub);
            }
            last_trajectory_num = trajectories.size();
            std::cout << "final time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
        }else{
            if(!local_goal_subscribed){
                std::cout << "waiting for local goal" << std::endl;
            }
            if(!local_map_updated){
                std::cout << "waiting for local map" << std::endl;
            }
            if(!odom_updated){
                std::cout << "waiting for odom" << std::endl;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void StateLatticePlannerROS::visualize_trajectories(const std::vector<MotionModelDiffDrive::Trajectory>& trajectories, const double r, const double g, const double b, const int trajectories_size, const ros::Publisher& pub)
{
    visualization_msgs::MarkerArray v_trajectories;
    int count = 0;
    const int size = trajectories.size();
    for(;count<size;count++){
        visualization_msgs::Marker v_trajectory;
        v_trajectory.header.frame_id = ROBOT_FRAME;
        v_trajectory.header.stamp = ros::Time::now();
        v_trajectory.color.r = r;
        v_trajectory.color.g = g;
        v_trajectory.color.b = b;
        v_trajectory.color.a = 0.8;
        v_trajectory.ns = pub.getTopic();
        v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
        v_trajectory.action = visualization_msgs::Marker::ADD;
        v_trajectory.lifetime = ros::Duration();
        v_trajectory.id = count;
        v_trajectory.pose.orientation.w = 1.0;
        v_trajectory.scale.x = 0.02;
        geometry_msgs::Point p;
        for(const auto& pose : trajectories[count].trajectory){
            p.x = pose(0);
            p.y = pose(1);
            v_trajectory.points.push_back(p);
        }
        v_trajectories.markers.push_back(v_trajectory);
    }
    for(;count<trajectories_size;){
        visualization_msgs::Marker v_trajectory;
        v_trajectory.header.frame_id = ROBOT_FRAME;
        v_trajectory.header.stamp = ros::Time::now();
        v_trajectory.ns = pub.getTopic();
        v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
        v_trajectory.action = visualization_msgs::Marker::DELETE;
        v_trajectory.lifetime = ros::Duration();
        v_trajectory.id = count;
        v_trajectories.markers.push_back(v_trajectory);
        count++;
    }
    pub.publish(v_trajectories);
}

void StateLatticePlannerROS::visualize_trajectory(const MotionModelDiffDrive::Trajectory& trajectory, const double r, const double g, const double b, const ros::Publisher& pub)
{
    visualization_msgs::Marker v_trajectory;
    v_trajectory.header.frame_id = ROBOT_FRAME;
    v_trajectory.header.stamp = ros::Time::now();
    v_trajectory.color.r = r;
    v_trajectory.color.g = g;
    v_trajectory.color.b = b;
    v_trajectory.color.a = 0.8;
    v_trajectory.ns = pub.getTopic();
    v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    v_trajectory.action = visualization_msgs::Marker::ADD;
    v_trajectory.lifetime = ros::Duration();
    v_trajectory.pose.orientation.w = 1.0;
    v_trajectory.pose.position.z = 0.1;
    v_trajectory.scale.x = 0.10;
    geometry_msgs::Point p;
    for(const auto& pose : trajectory.trajectory){
        p.x = pose(0);
        p.y = pose(1);
        v_trajectory.points.push_back(p);
    }
    pub.publish(v_trajectory);
}

template<typename TYPE>
void StateLatticePlannerROS::get_obstacle_map(const nav_msgs::OccupancyGrid& input_map, state_lattice_planner::ObstacleMap<TYPE>& output_map)
{
    output_map.set_shape(input_map.info.width, input_map.info.height, input_map.info.resolution);
    output_map.data.clear();
    for(const auto& data : input_map.data){
        output_map.data.emplace_back(data);
    }
}
