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

    SamplingParams sp(N_P, N_H, MAX_ALPHA, MAX_PSI);
    sampling_params = sp;

    velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    local_goal_sub = nh.subscribe("/local_goal", 1, &StateLatticePlanner::local_goal_callback, this);
    local_map_sub = nh.subscribe("/local_map", 1, &StateLatticePlanner::local_map_callback, this);
    odom_sub = nh.subscribe("/odom", 1, &StateLatticePlanner::odom_callback, this);

    local_goal_subscribed = false;
    local_map_updated = false;
    odom_updated = false;
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
    local_goal_subscribed = true;
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

void StateLatticePlanner::sample_states(const std::vector<double>& sample_angles, const SamplingParams& params, std::vector<Eigen::Vector3d>& states)
{
    /*
     * sample_angles: [0, 1]
     */
    std::vector<Eigen::Vector3d> _states;
    for(auto angle_ratio : sample_angles){
        double angle = params.min_alpha + (params.max_alpha - params.min_alpha) * angle_ratio;
        for(int i=0;i<params.n_h;i++){
            double x = params.length * cos(angle);
            double y = params.length * sin(angle);
            if(params.n_h > 0){
                double yaw = 0;
                if(params.n_h != 1){
                    double ratio = double(i) / (params.n_h - 1);
                    yaw = params.min_psi + (params.span_psi) * ratio + angle;
                }else{
                    yaw = (params.span_psi) * 0.5 + angle;
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

void StateLatticePlanner::generate_biased_polar_states(const int n_s, const Eigen::Vector3d& goal, const SamplingParams& params, std::vector<Eigen::Vector3d>& states)
{
    /*
     * n_s: param for biased polar sampling
     */
    // params.length is ignored in this function (distance for goal is used)
    SamplingParams _params = params;
    std::cout << "biased polar sampling" << std::endl;
    double alpha_coeff = _params.span_alpha / double(n_s - 1);
    std::vector<double> cnav;
    double goal_distance = goal.segment(0, 2).norm();
    for(int i=0;i<n_s;i++){
        double angle = _params.min_alpha + double(i) * alpha_coeff;
        _params.length = goal_distance;
        Eigen::Vector2d terminal;
        terminal <<  _params.length * cos(angle),
                     _params.length * sin(angle);
        double diff = (goal.segment(0, 2) - terminal).norm();
        cnav.push_back(diff);
    }
    double cnav_sum = 0;
    double cnav_max = 0;
    for(auto& alpha_s : cnav){
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

void StateLatticePlanner::generate_trajectories(const std::vector<Eigen::Vector3d>& boundary_states, const double velocity, const double angular_velocity, std::vector<MotionModelDiffDrive::Trajectory>& trajectories)
{
    for(auto boundary_state : boundary_states){
        TrajectoryGeneratorDiffDrive tg;
        MotionModelDiffDrive::ControlParams output;
        double k0 = angular_velocity / velocity;
        MotionModelDiffDrive::ControlParams init(MotionModelDiffDrive::VelocityParams(velocity, 0), MotionModelDiffDrive::CurvatureParams(k0, 0, 0, boundary_state.segment(0, 2).norm()));
        MotionModelDiffDrive::Trajectory trajectory;
        double cost = tg.generate_optimized_trajectory(boundary_state, init, 1e-1, 1e-1, N_S, output, trajectory);
        if(cost > 0){
            trajectories.push_back(trajectory);
        }
    }
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
        int xi = round((bresenhams_line[i](0) + local_costmap.info.origin.position.x) / resolution);
        int yi = round((bresenhams_line[i](1) + local_costmap.info.origin.position.y) / resolution);
        if(local_costmap.data[xi + local_costmap.info.width * yi] != 0){
            return true;
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
    for(auto& candidate_trajectory : candidate_trajectories){
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
        if(local_goal_subscribed && local_map_updated && odom_updated){
            Eigen::Vector3d goal(local_goal.pose.position.x, local_goal.pose.position.y, tf::getYaw(local_goal.pose.orientation));
            std::vector<Eigen::Vector3d> states;
            generate_biased_polar_states(N_S, goal, sampling_params, states);
            std::vector<MotionModelDiffDrive::Trajectory> trajectories;
            generate_trajectories(states, current_velocity.linear.x, current_velocity.angular.z, trajectories);
            std::vector<MotionModelDiffDrive::Trajectory> candidate_trajectories;
            for(auto& trajectory : trajectories){
                if(!check_collision(local_map, trajectory.trajectory)){
                    candidate_trajectories.push_back(trajectory);
                }
            }
            MotionModelDiffDrive::Trajectory trajectory;
            pickup_trajectory(candidate_trajectories, goal, trajectory); 

            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = trajectory.velocities[0];
            cmd_vel.angular.z = trajectory.angular_velocities[0];
            velocity_pub.publish(cmd_vel);

            local_map_updated = false;
            odom_updated = false;
        }else{
            if(!local_goal_subscribed){
                std::cout << "waiting for local goal" << std::endl;
            }
            if(!local_map_updated){
                std::cout << "waiting for local map" << std::endl;
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
    output.resize(size);
    std::vector<Eigen::Vector3d> bresenhams_line;
    for(int i=0;i<size-2;i++){
        double x0 = trajectory[i](0);
        double y0 = trajectory[i](1);
        double x1 = trajectory[i+1](0);
        double y1 = trajectory[i+1](1);

        bool steep = fabs(y1 - y0) > fabs(x1 - x0);

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
                output[i](0) = yt;
                output[i](1) = xt;
            }else{
                output[i](0) = xt;
                output[i](1) = yt;
            }
            error += delta_error;
            if(error >= 0.5){
                yt += y_step;
                error -= 1;
            }
        }

    }
}
