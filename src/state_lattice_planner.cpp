/**
* @file state_lattice_planner.cpp
* @author AMSL
*/
#include "state_lattice_planner/state_lattice_planner.h"

StateLatticePlanner::StateLatticePlanner(void)
{
    HZ = 10.0;
    MAX_ITERATION = 100;
    OPTIMIZATION_TOLERANCE = 0.1;
    MAX_ACCELERATION = 1.0;
    TARGET_VELOCITY = 0.8;
    MAX_YAWRATE = 1.0;
    MAX_D_YAWRATE = 3.14;
    MAX_WHEEL_ANGULAR_VELOCITY = 11.6;
    WHEEL_RADIUS = 0.125;
    TREAD = 0.5;
    VERBOSE = false;
    ENABLE_CONTROL_SPACE_SAMPLING = false;
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

void StateLatticePlanner::set_optimization_params(int max_iteration_, double tolerance_)
{
    MAX_ITERATION = std::max(max_iteration_, 0);
    OPTIMIZATION_TOLERANCE = std::max(tolerance_, 0.0);
}

void StateLatticePlanner::set_target_velocity(double v)
{
    TARGET_VELOCITY = v;
}

void StateLatticePlanner::set_motion_params(double max_acceleration_, double max_yawrate_, double max_d_yawrate_)
{
    MAX_ACCELERATION = max_acceleration_;
    MAX_YAWRATE = max_yawrate_;
    MAX_D_YAWRATE = max_d_yawrate_;
}

void StateLatticePlanner::set_vehicle_params(double wheel_radius_, double tread_)
{
    WHEEL_RADIUS = wheel_radius_;
    TREAD = tread_;
}

void StateLatticePlanner::set_sampling_params(const SamplingParams& sampling_params_)
{
    sampling_params = sampling_params_;
}

void StateLatticePlanner::load_lookup_table(const std::string& LOOKUP_TABLE_FILE_NAME)
{
    LookupTableUtils::load_lookup_table(LOOKUP_TABLE_FILE_NAME, lookup_table);
}

void StateLatticePlanner::sample_states(const std::vector<double>& sample_angles, std::vector<Eigen::Vector3d>& states)
{
    /*
     * sample_angles: [0, 1]
     */
    const auto params = sampling_params;
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
                yaw = atan2(sin(yaw), cos(yaw));
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
                yaw = atan2(sin(yaw), cos(yaw));
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

void StateLatticePlanner::generate_biased_polar_states(const int n_s, const Eigen::Vector3d& goal, double target_velocity, std::vector<Eigen::Vector3d>& states)
{
    /*
     * n_s: param for biased polar sampling
     */
    // params.length is ignored in this function (distance for goal is used)
    SamplingParams _params = sampling_params;
    double alpha_coeff = _params.span_alpha / double(n_s - 1);
    if(target_velocity < 0.0){
        _params.min_alpha = M_PI - sampling_params.max_alpha;
        _params.max_alpha = -sampling_params.min_alpha;
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
    int trajectory_num = boundary_states.size();
    std::vector<MotionModelDiffDrive::Trajectory> trajectories_(trajectory_num);
    #pragma omp parallel for
    for(int i=0;i<trajectory_num;i++){
        Eigen::Vector3d boundary_state = boundary_states[i];
        TrajectoryGeneratorDiffDrive tg;
        tg.set_verbose(VERBOSE);
        tg.set_motion_param(MAX_YAWRATE, MAX_D_YAWRATE, MAX_ACCELERATION, MAX_WHEEL_ANGULAR_VELOCITY, WHEEL_RADIUS, TREAD);
        MotionModelDiffDrive::ControlParams output;
        double k0 = angular_velocity;

        MotionModelDiffDrive::ControlParams param;
        LookupTableUtils::get_optimized_param_from_lookup_table(lookup_table, boundary_state, velocity, k0, param);
        // std::cout << "v0: " << velocity << ", " << "k0: " << k0 << ", " << "km: " << param.omega.km << ", " << "kf: " << param.omega.kf << ", " << "sf: " << param.omega.sf << std::endl;

        MotionModelDiffDrive::ControlParams init(MotionModelDiffDrive::VelocityParams(velocity, MAX_ACCELERATION, target_velocity, target_velocity, MAX_ACCELERATION)
                                               , MotionModelDiffDrive::AngularVelocityParams(k0, param.omega.km, param.omega.kf, param.omega.sf));

        MotionModelDiffDrive::Trajectory trajectory;
        // std::cout << boundary_state.transpose() << std::endl;
        double cost = tg.generate_optimized_trajectory(boundary_state, init, 1.0 / HZ, OPTIMIZATION_TOLERANCE, MAX_ITERATION, output, trajectory);
        // std::cout << trajectory.trajectory.back().transpose() << std::endl;
        if(cost > 0){
            trajectories_[i] = trajectory;
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
    if(ENABLE_CONTROL_SPACE_SAMPLING){
        int min_trajectory_size = trajectories_[0].trajectory.size();
        for(auto& traj : trajectories_){
            min_trajectory_size = std::min(min_trajectory_size, (int)traj.trajectory.size());
        }
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                MotionModelDiffDrive::Trajectory traj;
                MotionModelDiffDrive mmdd;
                mmdd.set_param(MAX_YAWRATE, MAX_D_YAWRATE, MAX_ACCELERATION, MAX_WHEEL_ANGULAR_VELOCITY, WHEEL_RADIUS, TREAD);
                MotionModelDiffDrive::State state(0, 0, 0, velocity, angular_velocity);
                traj.trajectory.emplace_back(Eigen::Vector3d(state.x, state.y, state.yaw));
                traj.velocities.emplace_back(state.v);
                traj.angular_velocities.emplace_back(state.omega);
                double velocity_ = velocity + (j - 1) * MAX_ACCELERATION / HZ;
                velocity_ = std::min(TARGET_VELOCITY, std::max(-TARGET_VELOCITY, velocity_));
                double omega = angular_velocity + (i - 1) * MAX_D_YAWRATE / HZ;
                omega = std::min(MAX_YAWRATE, std::max(omega, -MAX_YAWRATE));
                for(int j=0;j<min_trajectory_size;j++){
                    mmdd.update(state, velocity_, omega, 1.0 / HZ, state);
                    traj.trajectory.emplace_back(Eigen::Vector3d(state.x, state.y, state.yaw));
                    traj.velocities.emplace_back(state.v);
                    traj.angular_velocities.emplace_back(state.omega);
                }
                trajectories_.emplace_back(traj);
            }
        }
    }
    std::copy(trajectories_.begin(), trajectories_.end(), std::back_inserter(trajectories));
    if(trajectories.size() == 0){
        std::cout << "\033[91mERROR: no trajectory was generated\033[00m" << std::endl;
        return false;
    }
    return true;
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
            std::swap(x0, y0);
            std::swap(x1, y1);
        }
        if(x0 > x1){
            std::swap(x0, x1);
            std::swap(y0, y1);
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

double StateLatticePlanner::get_target_velocity(const Eigen::Vector3d& goal)
{
    double direction = atan2(goal(1), goal(0));
    if(fabs(direction) < M_PI * 0.75){
        return TARGET_VELOCITY;
    }else{
        return -TARGET_VELOCITY;
    }
}

bool StateLatticePlanner::check_collision(const state_lattice_planner::ObstacleMap<int>& map, const std::vector<Eigen::Vector3d>& trajectory)
{
    /*
     * if given trajectory is considered to collide with an obstacle, return true
     */
    double resolution = map.get_resolution();
    std::vector<Eigen::Vector3d> bresenhams_line;
    generate_bresemhams_line(trajectory, resolution, bresenhams_line);
    int size = bresenhams_line.size();
    for(int i=0;i<size;i++){
        int index = map.get_index_from_xy(bresenhams_line[i](0), bresenhams_line[i](1));
        if(map.data[index] != 0){
            return true;
        }
    }
    return false;
}

bool StateLatticePlanner::check_collision(const state_lattice_planner::ObstacleMap<int>& map, const std::vector<Eigen::Vector3d>& trajectory, double range)
{
    /*
     * if given trajectory is considered to collide with an obstacle, return true
     */
    double resolution = map.get_resolution();
    std::vector<Eigen::Vector3d> bresenhams_line;
    generate_bresemhams_line(trajectory, resolution, bresenhams_line);
    int size = bresenhams_line.size();
    for(int i=0;i<size;i++){
        int index = map.get_index_from_xy(bresenhams_line[i](0), bresenhams_line[i](1));
        if(map.data[index] != 0){
            return true;
        }
    }
    return false;
}
