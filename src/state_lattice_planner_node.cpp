#include "state_lattice_planner/state_lattice_planner.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "state_lattice_planner");
    StateLatticePlanner planner;
    planner.process();
    return 0;
}
