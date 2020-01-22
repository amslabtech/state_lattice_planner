#include "state_lattice_planner/lookup_table_generator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lookup_table_generator");
    LookupTableGenerator lookup_table_generator;
    std::string data = lookup_table_generator.process();
    lookup_table_generator.save(data);
    return 0;
}
