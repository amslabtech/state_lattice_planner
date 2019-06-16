#include "state_lattice_planner/lookup_table_generator.h"

LookupTableGenerator::LookupTableGenerator(void)
:local_nh("~")
{
    local_nh.param("MIN_X", MIN_X, {0.5});
    local_nh.param("MAX_X", MAX_X, {5.0});
    local_nh.param("DELTA_X", DELTA_X, {0.5});
    local_nh.param("MAX_Y", MAX_Y, {5.0});
    local_nh.param("DELTA_Y", DELTA_Y, {0.5});
    local_nh.param("MAX_YAW", MAX_YAW, {M_PI / 3.0});
    local_nh.param("DELTA_YAW", DELTA_YAW, {M_PI / 6.0});

}

void LookupTableGenerator::process(void)
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lookup_table_generator");
    LookupTableGenerator lookup_table_generator;
    lookup_table_generator.process();
    return 0;
}
