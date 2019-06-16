#include "state_lattice_planner/lookup_table_generator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lookup_table_generator");
    LookupTableGenerator lookup_table_generator;
    lookup_table_generator.process();
    return 0;
}
