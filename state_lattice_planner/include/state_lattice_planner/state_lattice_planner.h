#ifndef __STATE_LATTICE_PLANNER_H
#define __STATE_LATTICE_PLANNER_H

#include <ros/ros.h>

#include <Eigen/Dense>

#include <trajectory_generator/motion_model_diff_drive.h>
#include <trajectory_generator/trajectory_generator_diff_drive.h>

class StateLatticePlanner 
{
public:
    StateLatticePlanner(void);

    void process(void);

private:
    double HZ;
};

#endif //__STATE_LATTICE_PLANNER_H
