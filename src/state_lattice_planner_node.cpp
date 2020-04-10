#include "state_lattice_planner/state_lattice_planner_ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "state_lattice_planner");
    StateLatticePlannerROS planner;
    planner.process();
    return 0;
}
