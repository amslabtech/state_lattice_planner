#include "state_lattice_planner/trajectory_viewer.h"

TrajectoryViewer::TrajectoryViewer(void)
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
    local_nh.param("VERBOSE", VERBOSE, {false});
    local_nh.param("V0", V0, {0.0});
    local_nh.param("K0", K0, {0.0});

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
    std::cout << "VERBOSE: " << VERBOSE << std::endl;
    std::cout << "V0: " << V0 << std::endl;
    std::cout << "K0: " << K0 << std::endl;

    SamplingParams sp(N_P, N_H, MAX_ALPHA, MAX_PSI);
    sampling_params = sp;

    candidate_trajectories_pub = local_nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
    selected_trajectory_pub = local_nh.advertise<visualization_msgs::Marker>("selected_trajectory", 1);

    local_goal_sub = nh.subscribe("/local_goal", 1, &TrajectoryViewer::local_goal_callback, this);
    target_velocity_sub = nh.subscribe("/target_velocity", 1, &TrajectoryViewer::target_velocity_callback, this);

    local_goal_updated = false;

    LookupTableUtils::load_lookup_table(LOOKUP_TABLE_FILE_NAME, lookup_table);
}

TrajectoryViewer::SamplingParams::SamplingParams(void)
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

TrajectoryViewer::SamplingParams::SamplingParams(const int _n_p, const int _n_h, const double _length, const double _max_alpha, const double _max_psi)
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

TrajectoryViewer::SamplingParams::SamplingParams(const int _n_p, const int _n_h, const double _max_alpha, const double _max_psi)
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

void TrajectoryViewer::local_goal_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    local_goal = *msg;
    local_goal_updated = true;
}

void TrajectoryViewer::target_velocity_callback(const geometry_msgs::TwistConstPtr& msg)
{
    if(msg->linear.x > 0.0){
        TARGET_VELOCITY = msg->linear.x;
        std::cout << "\033[31mtarget velocity was updated to " << TARGET_VELOCITY << "[m/s]\033[0m" << std::endl;
    }
}

void TrajectoryViewer::sample_states(const std::vector<double>& sample_angles, const SamplingParams& params, std::vector<Eigen::Vector3d>& states)
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

void TrajectoryViewer::generate_biased_polar_states(const int n_s, const Eigen::Vector3d& goal, const SamplingParams& params, double target_velocity, std::vector<Eigen::Vector3d>& states)
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

bool TrajectoryViewer::generate_trajectories(const std::vector<Eigen::Vector3d>& boundary_states, const double velocity, const double angular_velocity, const double target_velocity, std::vector<MotionModelDiffDrive::Trajectory>& trajectories)
{
    std::cout << "generate trajectories to boundary states" << std::endl;
    int count = 0;
    int trajectory_num = boundary_states.size();
    std::vector<MotionModelDiffDrive::Trajectory> trajectories_(trajectory_num);
    #pragma omp parallel for
    for(int i=0;i<trajectory_num;i++){
        Eigen::Vector3d boundary_state = boundary_states[i];
        // double start = ros::Time::now().toSec();
        TrajectoryGeneratorDiffDrive tg;
        tg.set_verbose(VERBOSE);
        tg.set_motion_param(MAX_YAWRATE, MAX_D_YAWRATE, MAX_ACCELERATION, MAX_WHEEL_ANGULAR_VELOCITY, WHEEL_RADIUS, TREAD);
        MotionModelDiffDrive::ControlParams output;
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
            trajectories_[i] = trajectory;
            //std::cout << "generate time " << count << ": " << ros::Time::now().toSec() - start << "[s]" << std::endl;
            count++;
        }
    }
    for(auto it=trajectories_.begin();it!=trajectories_.end();){
        if(it->trajectory.size() == 0){
            it = trajectories_.erase(it);
        }else{
            ++it;
        }
    }
    std::copy(trajectories_.begin(), trajectories_.end(), std::back_inserter(trajectories));
    if(trajectories.size() == 0){
        std::cout << "\033[91mERROR: no trajectory was generated\033[00m" << std::endl;
        return false;
    }
    return true;
}

bool TrajectoryViewer::pickup_trajectory(const std::vector<MotionModelDiffDrive::Trajectory>& candidate_trajectories, const Eigen::Vector3d& goal, MotionModelDiffDrive::Trajectory& output)
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
    auto compare_trajectories = [=](const MotionModelDiffDrive::Trajectory& lhs, const MotionModelDiffDrive::Trajectory& rhs)
    {
        double l_distance = (lhs.trajectory.back().segment(0, 2) - goal.segment(0, 2)).norm();
        double r_distance = (rhs.trajectory.back().segment(0, 2) - goal.segment(0, 2)).norm();
        return l_distance < r_distance;
    };
    std::sort(trajectories.begin(), trajectories.end(), compare_trajectories);

    double min_diff_yaw = 100;
    const int N = ((int)trajectories.size() < sampling_params.n_h) ? trajectories.size() : sampling_params.n_h;
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

void TrajectoryViewer::process(void)
{
    ros::Rate loop_rate(HZ);

    while(ros::ok()){
        if(local_goal_updated){
            std::cout << "=== trajectory viewer ===" << std::endl;
            double start = ros::Time::now().toSec();
            int max_trajecotry_num = N_P * N_H;
            std::cout << "local goal: \n" << local_goal << std::endl;
            current_velocity.linear.x = V0;
            current_velocity.angular.z = K0;
            std::cout << "current_velocity: \n" << current_velocity << std::endl;
            Eigen::Vector3d goal(local_goal.pose.position.x, local_goal.pose.position.y, tf::getYaw(local_goal.pose.orientation));
            std::vector<Eigen::Vector3d> states;
            double target_velocity = get_target_velocity(goal);
            generate_biased_polar_states(N_S, goal, sampling_params, target_velocity, states);
            std::vector<MotionModelDiffDrive::Trajectory> trajectories;
            bool generated = generate_trajectories(states, current_velocity.linear.x, current_velocity.angular.z, target_velocity, trajectories);
            if(goal.segment(0, 2).norm() < 0.1){
                generated = false;
            }
            if(generated){
                visualize_trajectories(trajectories, 0, 1, 0, max_trajecotry_num, candidate_trajectories_pub);

                std::cout << "check candidate trajectories" << std::endl;
                std::vector<MotionModelDiffDrive::Trajectory> candidate_trajectories;
                for(const auto& trajectory : trajectories){
                    candidate_trajectories.push_back(trajectory);
                }
                std::cout << "trajectories: " << trajectories.size() << std::endl;
                std::cout << "candidate_trajectories: " << candidate_trajectories.size() << std::endl;
                if(candidate_trajectories.size() > 0){
                    std::cout << "pickup a optimal trajectory from candidate trajectories" << std::endl;
                    MotionModelDiffDrive::Trajectory trajectory;
                    pickup_trajectory(candidate_trajectories, goal, trajectory);
                    int size = trajectory.trajectory.size();
                    for(int i=0;i<size;i++){
                        std::cout << i << ": " << trajectory.trajectory[i].transpose() << ", " << trajectory.velocities[i] << "[m/s], " << trajectory.angular_velocities[i] << "[rad/s]" << std::endl;
                    }
                    visualize_trajectory(trajectory, 1, 0, 0, selected_trajectory_pub);
                    std::cout << "pickup time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
                }else{
                    std::cout << "\033[91mERROR: stacking\033[00m" << std::endl;
                    std::vector<MotionModelDiffDrive::Trajectory> clear_trajectories;
                    visualize_trajectories(clear_trajectories, 0, 1, 0, max_trajecotry_num, candidate_trajectories_pub);
                    visualize_trajectory(MotionModelDiffDrive::Trajectory(), 1, 0, 0, selected_trajectory_pub);
                }
            }else{
                std::cout << "\033[91mERROR: no optimized trajectory was generated\033[00m" << std::endl;
                std::cout << "\033[91mturn for local goal\033[00m" << std::endl;
                // for clear
                std::vector<MotionModelDiffDrive::Trajectory> clear_trajectories;
                visualize_trajectories(clear_trajectories, 0, 1, 0, max_trajecotry_num, candidate_trajectories_pub);
                visualize_trajectory(MotionModelDiffDrive::Trajectory(), 1, 0, 0, selected_trajectory_pub);
            }
            std::cout << "final time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
        }else{
            if(!local_goal_updated){
                std::cout << "\rwaiting for local goal";
            }
        }
        local_goal_updated = false;
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void TrajectoryViewer::swap(double& a, double& b)
{
    double temp = a;
    a = b;
    b = temp;
}

void TrajectoryViewer::visualize_trajectories(const std::vector<MotionModelDiffDrive::Trajectory>& trajectories, const double r, const double g, const double b, const int trajectories_size, const ros::Publisher& pub)
{
    visualization_msgs::MarkerArray v_trajectories;
    int count = 0;
    const int size = trajectories.size();
    for(;count<size;count++){
        visualization_msgs::Marker v_trajectory;
        v_trajectory.header.frame_id = ROBOT_FRAME;
        v_trajectory.header.stamp = ros::Time::now();
        v_trajectory.pose.orientation = tf::createQuaternionMsgFromYaw(0);
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
        v_trajectory.pose.orientation = tf::createQuaternionMsgFromYaw(0);
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

void TrajectoryViewer::visualize_trajectory(const MotionModelDiffDrive::Trajectory& trajectory, const double r, const double g, const double b, const ros::Publisher& pub)
{
    visualization_msgs::Marker v_trajectory;
    v_trajectory.header.frame_id = ROBOT_FRAME;
    v_trajectory.header.stamp = ros::Time::now();
    v_trajectory.pose.orientation = tf::createQuaternionMsgFromYaw(0);
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

double TrajectoryViewer::get_target_velocity(const Eigen::Vector3d& goal)
{
    double direction = atan2(goal(1), goal(0));
    if(fabs(direction) < M_PI * 0.75){
        return TARGET_VELOCITY;
    }else{
        return -TARGET_VELOCITY;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_viewer");
    TrajectoryViewer tv;
    tv.process();
    return 0;
}
