// Copyright 2019 amsl

#ifndef STATE_LATTICE_PLANNER_LOOKUP_TABLE_UTILS_H
#define STATE_LATTICE_PLANNER_LOOKUP_TABLE_UTILS_H

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "trajectory_generator/motion_model_diff_drive.h"

namespace LookupTableUtils
{
class StateWithControlParams
{
public:
  StateWithControlParams(void);

  Eigen::Vector3d state;  // x, y, yaw
  MotionModelDiffDrive::ControlParams control;

private:
};

typedef std::map<double, std::map<double, std::vector<StateWithControlParams>>>
    LookupTable;

bool load_lookup_table(const std::string&, LookupTable&);

void get_optimized_param_from_lookup_table(
    const LookupTable&, const Eigen::Vector3d, const double, const double,
    MotionModelDiffDrive::ControlParams&);
}  // namespace LookupTableUtils

#endif  // STATE_LATTICE_PLANNER_LOOKUP_TABLE_UTILS_H
