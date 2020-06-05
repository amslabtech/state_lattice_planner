# state_lattice_planner

[![Build Status](https://travis-ci.org/amslabtech/state_lattice_planner.svg?branch=master)](https://travis-ci.org/amslabtech/state_lattice_planner)
![issue_opened](https://img.shields.io/github/issues/amslabtech/state_lattice_planner.svg)
![issue_closed](https://img.shields.io/github/issues-closed/amslabtech/state_lattice_planner.svg)


## Overview
TBW

The API documantation is [here](https://amslabtech.github.io/state_lattice_planner/).

## Enviornment
- Ubuntu 16.04 or 18.04
- ROS Kinetic or Melodic

## Install and Build

```
cd catkin_workspace/src
git clone https://github.com/amslabtech/state_lattice_planner.git
cd ..
catkin_make
```

## Nodes
### state_lattice_planner
- local planner node
#### Published topics
- /cmd_vel (geometry_msgs/Twist)
- ~/candidate_trajectoryies (visualization_msgs/MarkerArray)
  - for visualization
- ~/candidate_trajectoryies/no_collision (visualization_msgs/MarkerArray)
  - for visualization

#### Subscribed topics
- /local_goal (geometry_msgs/PoseStamped)
  - the local goal must be in the local map
- /local_map (nav_msgs/OccupancyGrid)
  - robot-centered costmap
- /odom (nav_msgs/Odometry)
  - robot odometry

#### Parameters
- HZ
  - main loop rate (default: 20[Hz])
- ROBOT_FRAME
  - robot's coordinate frame (default: base_link)
- N_P
  - number of terminal state sampling for x-y position (default: 10)
- N_H
  - number of terminal state sampling for heading direction (default: 3)
- MAX_ALPHA
  - max terminal state sampling direction (default: M_PI/3.0[rad/s])
- MAX_PSI
  - max heading direction at terminal state (default: M_PI/6.0[rad/s])
- N_S
  - parameter for globally guided sampling (default: 1000)
- MAX_ACCELERATION
  - max acceleration of robot (absolute value)(default: 1.0[m/ss])
- TARGET_VELOCITY
  - max velocity of robot's target velocity (default: 0.8[m/s])
- LOOKUP_TABLE_FILE_NAME
  - absolute path of lookup table (default: $HOME/lookup_table.csv)
- MAX_ITERATION
  - max number of iteration (default: 100)
- OPTIMIZATION_TOLERANCE
  - when the cost becomes lower than this parameter, optimization loop is finished (default: 0.1)
- MAX_CURVATURE
  - max trajectory curvature (default: 1.0[rad/m])
- MAX_D_CURVATURE
  - max time derivative of trajectory curvature (default: 2.0[rad/ms]
- MAX_YAWRATE
  - max robot's yawrate (default: 0.8[rad/s])
  
#### Runtime requirement
- TF (from /odom to /base_link) is required

### lookup_table_generator
- this node is a tool for generating a lookup table, not for planning. so this node doesn't publish or subscribe topics.
#### Parameters
- MIN_X
  - target state sampling parameter (default: 1.0[m])
- MAX_X
  - target state sampling parameter (default: 7.0[m])
- DELTA_X
  - target state sampling parameter (default: 1.0[m])
- MAX_Y
  - target state sampling parameter (default: 3.0[m])
- DELTA_Y
  - target state sampling parameter (default: 1.0[m])
- MAX_YAW
  - target state sampling parameter (default: 1.0471975[rad])
- DELTA_YAW
  - target state sampling parameter (default: 1.0471975[rad])
- LOOKUP_TABLE_FILE_NAME
  - full path of lookup_table.csv
- MIN_V
  - initial velocity sampling parameter (default: 0.1[m/s])
- MAX_V
  - initial velocity sampling parameter (default: 0.8[m/s])
- DELTA_V
  - initial velocity sampling parameter (default: 0.1[m/s])
- MAX_KAPPA
  - initial curvature sampling parameter (default: 1.0[rad/m])
- DELTA_KAPPA
  - initial curvature sampling parameter (default: 0.2[rad/m])
- MAX_ACCELERATION
  - max acceleration of robot (default: 1.0[m/ss])
- MAX_CURVATURE
  - max trajectory curvature (default: 1.0[rad/m])
- MAX_D_CURVATURE
  - max time derivative of trajectory curvature (default: 2.0[rad/ms])
- MAX_YAWRATE
  - max yawrate of robot (default: 0.8[rad/s])

## How to Use
- for generating lookup table
```
roslaunch state_lattice_planner generate_lookup_table.launch
```
- for local path planning
```
roslaunch state_lattice_planner local_planner.launch
```

## References
- https://www.ri.cmu.edu/publications/state-space-sampling-of-feasible-motions-for-high-performance-mobile-robot-navigation-in-complex-environments/
- https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/StateLatticePlanner

