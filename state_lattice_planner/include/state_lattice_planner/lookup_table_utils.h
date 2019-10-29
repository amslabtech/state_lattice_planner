#ifndef __LOOKUP_TABLE_UTILS_H
#define __LOOKUP_TABLE_UTILS_H

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <Eigen/Dense>

#include "trajectory_generator/motion_model_diff_drive.h"

namespace LookupTableUtils
{
    class StateWithControlParams
    {
    public:
        StateWithControlParams(void);

        Eigen::Vector3d state;// x, y, yaw
        MotionModelDiffDrive::ControlParams control;
    private:
    };

    typedef std::map<double, std::map<double, std::vector<StateWithControlParams> > > LookupTable;

    bool load_lookup_table(const std::string&, LookupTable&);

    void get_optimized_param_from_lookup_table(const LookupTable&, const Eigen::Vector3d, const double, const double, MotionModelDiffDrive::ControlParams&);
}

#endif// __LOOKUP_TABLE_UTILS_H
