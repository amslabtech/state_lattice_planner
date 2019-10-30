#include "state_lattice_planner/state_lattice_planner.h"

StateLatticePlanner::StateLatticePlanner(void)
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

    SamplingParams sp(N_P, N_H, MAX_ALPHA, MAX_PSI);
    sampling_params = sp;

    velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    candidate_trajectories_pub = local_nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
    candidate_trajectories_no_collision_pub = local_nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories/no_collision", 1);
    selected_trajectory_pub = local_nh.advertise<visualization_msgs::Marker>("selected_trajectory", 1);

    local_goal_sub = nh.subscribe("/local_goal", 1, &StateLatticePlanner::local_goal_callback, this);
    local_map_sub = nh.subscribe("/local_map", 1, &StateLatticePlanner::local_map_callback, this);
    odom_sub = nh.subscribe("/odom", 1, &StateLatticePlanner::odom_callback, this);
    target_velocity_sub = nh.subscribe("/target_velocity", 1, &StateLatticePlanner::target_velocity_callback, this);

    local_goal_subscribed = false;
    local_map_updated = false;
    odom_updated = false;

    LookupTableUtils::load_lookup_table(LOOKUP_TABLE_FILE_NAME, lookup_table);
}

StateLatticePlanner::SamplingParams::SamplingParams(void)
{
    n_p = 0.0;
    n_h = 0.0;
    length = 0.0;
    max_alpha = 0.0;
    min_alpha = 0.0;
    span_alpha = max_alpha - min_alpha;
    max_psi = 0.0;
    min_psi = 0.0;
    span_psi = 0.0;
}

StateLatticePlanner::SamplingParams::SamplingParams(const int _n_p, const int _n_h, const double _length, const double _max_alpha, const double _max_psi)
{
    n_p = _n_p;
    n_h = _n_h;
    length = _length;
    max_alpha = _max_alpha;
    min_alpha = -_max_alpha;
    span_alpha = max_alpha - min_alpha;
    max_psi = _max_psi;
    min_psi = -_max_psi;
    span_psi = max_psi - min_psi;
}

StateLatticePlanner::SamplingParams::SamplingParams(const int _n_p, const int _n_h, const double _max_alpha, const double _max_psi)
{
    n_p = _n_p;
    n_h = _n_h;
    length = 0.0;
    max_alpha = _max_alpha;
    min_alpha = -_max_alpha;
    span_alpha = max_alpha - min_alpha;
    max_psi = _max_psi;
    min_psi = -_max_psi;
    span_psi = max_psi - min_psi;
}

void StateLatticePlanner::local_goal_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    local_goal = *msg;
    try{
        listener.transformPose("/odom", ros::Time(0), local_goal, local_goal.header.frame_id, local_goal);
        local_goal_subscribed = true;
    }catch(tf::TransformException ex){
        std::cout << ex.what() << std::endl;
    }
}

void StateLatticePlanner::local_map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    local_map = *msg;
    local_map_updated = true;
}

void StateLatticePlanner::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    current_velocity = msg->twist.twist;
    odom_updated = true;
}

void StateLatticePlanner::target_velocity_callback(const geometry_msgs::TwistConstPtr& msg)
{
    if(msg->linear.x > 0.0){
        TARGET_VELOCITY = msg->linear.x;
        std::cout << "\033[31mtarget velocity was updated to " << TARGET_VELOCITY << "[m/s]\033[0m" << std::endl;
    }
}

void StateLatticePlanner::sample_states(const std::vector<double>& sample_angles, const SamplingParams& params, std::vector<Eigen::Vector3d>& states)
{
    /*
     * sample_angles: [0, 1]
     */
    std::vector<Eigen::Vector3d> _states;
    for(auto angle_ratio : sample_angles){
        double angle = params.min_alpha + params.span_alpha * angle_ratio;
        double x = params.length * cos(angle);
        double y = params.length * sin(angle);
        double base_angle = angle;
        if(params.min_alpha > params.max_alpha){
            base_angle -= M_PI;
            base_angle = atan2(sin(base_angle), cos(base_angle));
        }
        for(int i=0;i<params.n_h;i++){
            if(params.n_h > 0){
                double yaw = 0;
                if(params.n_h != 1){
                    double ratio = double(i) / (params.n_h - 1);
                    yaw = params.min_psi + (params.span_psi) * ratio + base_angle;
                }else{
                    yaw = (params.span_psi) * 0.5 + base_angle;
                }
                Eigen::Vector3d state(x, y, yaw);
                _states.push_back(state);
            }else{
                std::cout << "sampling param error" << std::endl;
                exit(-1);
            }
        }
    }
    states = _states;
}

void StateLatticePlanner::generate_biased_polar_states(const int n_s, const Eigen::Vector3d& goal, const SamplingParams& params, double target_velocity, std::vector<Eigen::Vector3d>& states)
{
    /*
     * n_s: param for biased polar sampling
     */
    // params.length is ignored in this function (distance for goal is used)
    SamplingParams _params = params;
    double alpha_coeff = _params.span_alpha / double(n_s - 1);
    if(target_velocity < 0){
        _params.min_alpha = M_PI - _params.max_alpha;
        _params.max_alpha = -_params.min_alpha;
    }
    std::cout << "biased polar sampling" << std::endl;
    std::vector<double> cnav;
    double goal_distance = goal.segment(0, 2).norm();
    for(int i=0;i<n_s;i++){
        double angle = _params.min_alpha + double(i) * alpha_coeff;
        angle = atan2(sin(angle), cos(angle));
        _params.length = goal_distance;
        Eigen::Vector2d terminal;
        terminal <<  _params.length * cos(angle),
                     _params.length * sin(angle);
        double diff = (goal.segment(0, 2) - terminal).norm();
        cnav.push_back(diff);
    }
    double cnav_sum = 0;
    double cnav_max = 0;
    for(const auto& alpha_s : cnav){
        cnav_sum += alpha_s;
        if(cnav_max < alpha_s){
            cnav_max = alpha_s;
        }
    }
    // normalize
    //std::cout << "normalize" << std::endl;
    for(auto& alpha_s : cnav){
        alpha_s = (cnav_max - alpha_s) / (cnav_max * n_s - cnav_sum);
        //std::cout << alpha_s << std::endl;
    }
    // cumsum
    //std::cout << "cumsum" << std::endl;
    std::vector<double> cnav2;
    double cumsum = 0;
    std::vector<double> cumsum_list;
    for(auto cnav_it=cnav.begin();cnav_it!=cnav.end()-1;++cnav_it){
        cumsum += *cnav_it;
        cnav2.push_back(cumsum);
        //std::cout << cumsum << std::endl;
    }

    // sampling
    std::vector<double> biased_angles;
    for(int i=0;i<_params.n_p;i++){
        double sample_angle = double(i) / (_params.n_p - 1);
        //std::cout << "sample angle: " << sample_angle << std::endl;
        int count = 0;
        for(;count<n_s-1;count++){
            // if this loop finish without break, count is n_s - 1
            if(cnav2[count] >= sample_angle){
                break;
            }
        }
        //std::cout << "count: " << count << std::endl;
        biased_angles.push_back(count / double(n_s - 1));
    }
    //std::cout << "biased angles" << std::endl;
    /*
    for(auto angle : biased_angles){
        std::cout << angle << std::endl;
    }
    */
    sample_states(biased_angles, _params, states);
}

bool StateLatticePlanner::generate_trajectories(const std::vector<Eigen::Vector3d>& boundary_states, const double velocity, const double angular_velocity, const double target_velocity, std::vector<MotionModelDiffDrive::Trajectory>& trajectories)
{
    std::cout << "generate trajectories to boundary states" << std::endl;
    int count = 0;
    for(auto boundary_state : boundary_states){
        // double start = ros::Time::now().toSec();
        TrajectoryGeneratorDiffDrive tg;
        tg.set_verbose(VERBOSE);
        tg.set_motion_param(MAX_YAWRATE, MAX_D_YAWRATE, MAX_ACCELERATION, MAX_WHEEL_ANGULAR_VELOCITY, WHEEL_RADIUS, TREAD);
        MotionModelDiffDrive::ControlParams output;
        //std::cout << "v: " << velocity << ", " << "w: " << angular_velocity << std::endl;
        // double _velocity = velocity;
        // double k0 = angular_velocity / _velocity;
        // if(fabs(_velocity) < 1e-3){
        //     _velocity = 1e-3 * ((_velocity > 0) ? 1 : -1);
        //     // cheat
        //     k0 = 0;
        // }
        double k0 = angular_velocity;

        MotionModelDiffDrive::ControlParams param;
        LookupTableUtils::get_optimized_param_from_lookup_table(lookup_table, boundary_state, velocity, k0, param);
        // std::cout << "v0: " << velocity << ", " << "k0: " << k0 << ", " << "km: " << param.omega.km << ", " << "kf: " << param.omega.kf << ", " << "sf: " << param.omega.sf << std::endl;
        //std::cout << "lookup table time " << count << ": " << ros::Time::now().toSec() - start << "[s]" << std::endl;

        MotionModelDiffDrive::ControlParams init(MotionModelDiffDrive::VelocityParams(velocity, MAX_ACCELERATION, target_velocity, target_velocity, MAX_ACCELERATION)
                                               , MotionModelDiffDrive::AngularVelocityParams(k0, param.omega.km, param.omega.kf, param.omega.sf));

        MotionModelDiffDrive::Trajectory trajectory;
        // std::cout << boundary_state.transpose() << std::endl;
        double cost = tg.generate_optimized_trajectory(boundary_state, init, 1.0 / HZ, OPTIMIZATION_TOLERANCE, MAX_ITERATION, output, trajectory);
        // std::cout << trajectory.trajectory.back().transpose() << std::endl;
        if(cost > 0){
            trajectories.push_back(trajectory);
            //std::cout << "generate time " << count << ": " << ros::Time::now().toSec() - start << "[s]" << std::endl;
            count++;
        }
    }
    if(trajectories.size() == 0)
    {
        std::cout << "\033[91mERROR: no trajectory was generated\033[00m" << std::endl;
        return false;
    }
    return true;
}

bool StateLatticePlanner::check_collision(const nav_msgs::OccupancyGrid& local_costmap, const std::vector<Eigen::Vector3d>& trajectory)
{
    /*
     * if given trajectory is considered to collide with an obstacle, return true
     */
    double resolution = local_costmap.info.resolution;
    std::vector<Eigen::Vector3d> bresenhams_line;
    generate_bresemhams_line(trajectory, resolution, bresenhams_line);
    int size = bresenhams_line.size();
    for(int i=0;i<size;i++){
        int xi = round((bresenhams_line[i](0) - local_costmap.info.origin.position.x) / resolution);
        int yi = round((bresenhams_line[i](1) - local_costmap.info.origin.position.y) / resolution);
        //std::cout << xi << ", " << yi << std::endl;
        if(local_costmap.data[xi + local_costmap.info.width * yi] != 0){
            return true;
        }
    }
    return false;
}

bool StateLatticePlanner::check_collision(const nav_msgs::OccupancyGrid& local_costmap, const std::vector<Eigen::Vector3d>& trajectory, double range)
{
    /*
     * if given trajectory is considered to collide with an obstacle, return true
     */
    double resolution = local_costmap.info.resolution;
    std::vector<Eigen::Vector3d> bresenhams_line;
    generate_bresemhams_line(trajectory, resolution, bresenhams_line);
    int size = bresenhams_line.size();
    for(int i=0;i<size;i++){
        int xi = round((bresenhams_line[i](0) - local_costmap.info.origin.position.x) / resolution);
        int yi = round((bresenhams_line[i](1) - local_costmap.info.origin.position.y) / resolution);
        //std::cout << xi << ", " << yi << std::endl;
        if(local_costmap.data[xi + local_costmap.info.width * yi] != 0){
            if(bresenhams_line[i].norm() > range){
                return false;
            }else{
                return true;
            }
        }
    }
    return false;
}

bool StateLatticePlanner::pickup_trajectory(const std::vector<MotionModelDiffDrive::Trajectory>& candidate_trajectories, const Eigen::Vector3d& goal, MotionModelDiffDrive::Trajectory& output)
{
    /*
     * outputs a trajectory that is nearest to the goal
     */
    std::vector<MotionModelDiffDrive::Trajectory> trajectories;
    for(const auto& candidate_trajectory : candidate_trajectories){
        MotionModelDiffDrive::Trajectory traj;
        traj = candidate_trajectory;
        traj.cost = (candidate_trajectory.trajectory.back().segment(0, 2) - goal.segment(0, 2)).norm();
        trajectories.push_back(traj);
    }
    // ascending sort by cost
    std::sort(trajectories.begin(), trajectories.end());

    double min_diff_yaw = 100;
    const int N = ((trajectories.size() < sampling_params.n_h) ? trajectories.size() : sampling_params.n_h);
    for(int i=0;i<N;i++){
        double diff_yaw = trajectories[i].trajectory.back()(2) - goal(2);
        diff_yaw = fabs(atan2(sin(diff_yaw), cos(diff_yaw)));
        if(min_diff_yaw > diff_yaw){
            min_diff_yaw = diff_yaw;
            output = trajectories[i];
        }
    }
    if(!(output.trajectory.size() > 0)){
        return false;
    }
    return true;
}

void StateLatticePlanner::process(void)
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
            std::cout << "local goal: \n" << local_goal_base_link << std::endl;
            std::cout << "current_velocity: \n" << current_velocity << std::endl;
            Eigen::Vector3d goal(local_goal_base_link.pose.position.x, local_goal_base_link.pose.position.y, tf::getYaw(local_goal_base_link.pose.orientation));
            std::vector<Eigen::Vector3d> states;
            double target_velocity = get_target_velocity(goal);
            generate_biased_polar_states(N_S, goal, sampling_params, target_velocity, states);
            std::vector<MotionModelDiffDrive::Trajectory> trajectories;
            bool generated = generate_trajectories(states, current_velocity.linear.x, current_velocity.angular.z, target_velocity, trajectories);
            if(goal.segment(0, 2).norm() < 0.1){
                generated = false;
            }
            if(generated){
                visualize_trajectories(trajectories, 0, 1, 0, N_P * N_H, candidate_trajectories_pub);

                std::cout << "check candidate trajectories" << std::endl;
                std::vector<MotionModelDiffDrive::Trajectory> candidate_trajectories;
                for(const auto& trajectory : trajectories){
                    if(!check_collision(local_map, trajectory.trajectory)){
                        candidate_trajectories.push_back(trajectory);
                    }
                }
                std::cout << "trajectories: " << trajectories.size() << std::endl;
                std::cout << "candidate_trajectories: " << candidate_trajectories.size() << std::endl;
                if(candidate_trajectories.empty()){
                    // if no candidate trajectories
                    // collision checking with relaxed restrictions
                    for(const auto& trajectory : trajectories){
                        if(!check_collision(local_map, trajectory.trajectory, IGNORABLE_OBSTACLE_RANGE)){
                            candidate_trajectories.push_back(trajectory);
                        }
                    }
                    std::cout << "candidate_trajectories(ignore far obstacles): " << candidate_trajectories.size() << std::endl;
                }
                // std::cout << "candidate time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
                if(candidate_trajectories.size() > 0){
                    visualize_trajectories(candidate_trajectories, 0, 0.5, 1, N_P * N_H, candidate_trajectories_no_collision_pub);

                    std::cout << "pickup a optimal trajectory from candidate trajectories" << std::endl;
                    MotionModelDiffDrive::Trajectory trajectory;
                    pickup_trajectory(candidate_trajectories, goal, trajectory);
                    visualize_trajectory(trajectory, 1, 0, 0, selected_trajectory_pub);
                    std::cout << "pickup time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;

                    // int size = trajectory.trajectory.size();
                    // for(int i=0;i<size;i++){
                    //     std::cout << trajectory.trajectory[i].transpose() << ", " << trajectory.velocities[i] << "[m/s], " << trajectory.angular_velocities[i] << "[rad/s]" << std::endl;
                    // }

                    std::cout << "publish velocity" << std::endl;
                    geometry_msgs::Twist cmd_vel;
                    double calculation_time = ros::Time::now().toSec() - start;
                    int delayed_control_index = std::ceil(calculation_time * HZ) + 1;
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
                    visualize_trajectories(clear_trajectories, 0, 1, 0, N_P * N_H, candidate_trajectories_pub);
                    visualize_trajectories(clear_trajectories, 0, 0.5, 1, N_P * N_H, candidate_trajectories_no_collision_pub);
                    visualize_trajectory(MotionModelDiffDrive::Trajectory(), 1, 0, 0, selected_trajectory_pub);
                }
            }else{
                std::cout << "\033[91mERROR: no optimized trajectory was generated\033[00m" << std::endl;
                std::cout << "\033[91mturn for local goal\033[00m" << std::endl;
                double relative_direction = atan2(local_goal_base_link.pose.position.y, local_goal_base_link.pose.position.x);
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = 0;
                // cmd_vel.angular.z =  0.2 * ((relative_direction > 0) ? 1 : -1);
                cmd_vel.angular.z = std::min(std::max(goal(2), -MAX_YAWRATE), MAX_YAWRATE);
                velocity_pub.publish(cmd_vel);
                // for clear
                std::vector<MotionModelDiffDrive::Trajectory> clear_trajectories;
                visualize_trajectories(clear_trajectories, 0, 1, 0, N_P * N_H, candidate_trajectories_pub);
                visualize_trajectories(clear_trajectories, 0, 0.5, 1, N_P * N_H, candidate_trajectories_no_collision_pub);
                visualize_trajectory(MotionModelDiffDrive::Trajectory(), 1, 0, 0, selected_trajectory_pub);
            }
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

void StateLatticePlanner::swap(double& a, double& b)
{
    double temp = a;
    a = b;
    b = temp;
}

void StateLatticePlanner::generate_bresemhams_line(const std::vector<Eigen::Vector3d>& trajectory, const double& resolution, std::vector<Eigen::Vector3d>& output)
{
    int size = trajectory.size();
    output.clear();
    // maybe too much
    output.reserve(size * 2);
    for(int i=0;i<size-2;i++){
        double x0 = trajectory[i](0);
        double y0 = trajectory[i](1);
        double x1 = trajectory[i+1](0);
        double y1 = trajectory[i+1](1);

        bool steep = (fabs(y1 - y0) > fabs(x1 - x0));

        if(steep){
            swap(x0, y0);
            swap(x1, y1);
        }
        if(x0 > x1){
            swap(x0, x1);
            swap(y0, y1);
        }

        double delta_x = x1 - x0;
        double delta_y = fabs(y1 - y0);
        double error = 0;
        double delta_error = delta_y / delta_x;
        double y_step;
        double yt = y0;

        y_step = (y0 < y1) ? resolution : -resolution;

        for(double xt=x0;xt<x1;xt+=resolution){
            if(steep){
                Eigen::Vector3d p(yt, xt, 0);
                output.push_back(p);
            }else{
                Eigen::Vector3d p(xt, yt, 0);
                output.push_back(p);
            }
            error += delta_error;
            if(error >= 0.5){
                yt += y_step;
                error -= 1;
            }
        }
    }
}

void StateLatticePlanner::visualize_trajectories(const std::vector<MotionModelDiffDrive::Trajectory>& trajectories, const double r, const double g, const double b, const int trajectories_size, const ros::Publisher& pub)
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

void StateLatticePlanner::visualize_trajectory(const MotionModelDiffDrive::Trajectory& trajectory, const double r, const double g, const double b, const ros::Publisher& pub)
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
    v_trajectory.scale.x = 0.05;
    geometry_msgs::Point p;
    for(const auto& pose : trajectory.trajectory){
        p.x = pose(0);
        p.y = pose(1);
        v_trajectory.points.push_back(p);
    }
    pub.publish(v_trajectory);
}

double StateLatticePlanner::get_target_velocity(const Eigen::Vector3d& goal)
{
    double direction = atan2(goal(1), goal(0));
    if(fabs(direction) < M_PI * 0.75){
        return TARGET_VELOCITY;
    }else{
        return -TARGET_VELOCITY;
    }
}
