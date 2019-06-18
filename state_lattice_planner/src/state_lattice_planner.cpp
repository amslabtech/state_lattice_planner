#include "state_lattice_planner/state_lattice_planner.h"

StateLatticePlanner::StateLatticePlanner(void)
:local_nh("~")
{
    local_nh.param("HZ", HZ, {20});

    velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    local_goal_sub = nh.subscribe("/local_goal", 1, &StateLatticePlanner::local_goal_callback, this);
    local_map_sub = nh.subscribe("/local_map", 1, &StateLatticePlanner::local_map_callback, this);

    local_goal_subscribed = false;
    local_map_updated = false;
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

void StateLatticePlanner::generate_biased_polar_states(const int n_s, const double goal_direction, const SamplingParams& params, std::vector<Eigen::Vector3d>& states)
{
    /*
     * n_s: param for biased polar sampling
     * goal_direction: goal direction from robot [rad]
     */
    std::cout << "biased polar sampling" << std::endl;
    double alpha_coeff = params.span_alpha / double(n_s - 1);
    std::vector<double> cnav;
    for(int i=0;i<n_s;i++){
        double angle = params.min_alpha + double(i) * alpha_coeff;
        cnav.push_back(angle);
    }
    double cnav_sum = 0;
    double cnav_max = 0;
    for(auto& alpha_s : cnav){
        alpha_s = fabs(alpha_s - goal_direction);
        cnav_sum += alpha_s;
        if(cnav_max < alpha_s){
            cnav_max = alpha_s;
        }
    }
    // normalize
    std::cout << "normalize" << std::endl;
    for(auto& alpha_s : cnav){
        alpha_s = (cnav_max - alpha_s) / (cnav_max * n_s - cnav_sum);
        std::cout << alpha_s << std::endl;
    }
    // cumsum
    std::cout << "cumsum" << std::endl;
    std::vector<double> cnav2;
    double cumsum = 0;
    std::vector<double> cumsum_list;
    for(auto cnav_it=cnav.begin();cnav_it!=cnav.end()-1;++cnav_it){
        cumsum += *cnav_it;
        cnav2.push_back(cumsum);
        std::cout << cumsum << std::endl;
    }

    // sampling
    std::vector<double> biased_angles;
    for(int i=0;i<params.n_p;i++){
        double sample_angle = double(i) / (params.n_p - 1);
        std::cout << "sample angle: " << sample_angle << std::endl;
        int count = 0;
        for(;count<n_s-1;count++){
            // if this loop finish without break, count is n_s - 1
            if(cnav2[count] >= sample_angle){
                break;
            }
        }
        std::cout << "count: " << count << std::endl;
        biased_angles.push_back(count / double(n_s - 1));
    }
    std::cout << "biased angles" << std::endl;
    for(auto angle : biased_angles){
        std::cout << angle << std::endl;
    }
    sample_states(biased_angles, params, states);
}

void StateLatticePlanner::process(void)
{
    ros::Rate loop_rate(HZ);

    while(ros::ok()){
        if(local_goal_subscribed && local_map_updated){

        }else{
            if(!local_goal_subscribed){
                std::cout << "waiting for local goal" << std::endl;
            }
            if(!local_map_updated){
                std::cout << "waiting for local map" << std::endl;
            }
        }
        loop_rate.sleep();
    }
}
