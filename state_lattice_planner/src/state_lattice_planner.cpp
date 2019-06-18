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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "state_lattice_planner");
    StateLatticePlanner planner;
    planner.process();
    return 0;
}
