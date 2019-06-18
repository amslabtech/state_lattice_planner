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
    max_psi = _max_psi;
    max_psi = -_max_psi;
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
        double angle = params.min_alpha * (params.max_alpha - params.min_alpha) * angle_ratio;
        for(int i=0;i<params.n_h;i++){
            double x = params.length * cos(angle);
            double y = params.length * sin(angle);
            if(params.n_h > 0){
                double yaw = 0;
                if(params.n_h != 1){
                    double ratio = double(i) / (params.n_h - 1);
                    yaw = params.min_psi + (params.max_psi - params.min_psi) * ratio + angle;
                }else{
                    yaw = (params.max_psi - params.min_psi) * 0.5 + angle;
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

void StateLatticePlanner::generate_biased_polar_state()
{

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
