# State_lattice_planner

[![Build Status](https://travis-ci.org/amslabtech/state_lattice_planner.svg?branch=master)](https://travis-ci.org/amslabtech/state_lattice_planner)
![issue_opened](https://img.shields.io/github/issues/amslabtech/state_lattice_planner.svg)
![issue_closed](https://img.shields.io/github/issues-closed/amslabtech/state_lattice_planner.svg)

## Enviornment
- Ubuntu 16.04 or 18.04
- ROS Kinetic or Melodic

## Nodes
### State Lattice Planner
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
  - when the cost becomes lower than this parameter, optimization loop is finished 
- SHORTENING_TRAJECTORY_LENGTH_STEP
  - experimental feature
- SHORTENING_TRAJECTORY_MIN_LENGTH
  - experimental feautre
- MAX_CURVATURE
  - max trajectory curvature (default: [rad/m])
- MAX_D_CURVATURE
  - max time derivative of trajectory curvature (default: [rad/ms]
- MAX_YAWRATE 
  - max robot's yawrate (default: 1.0[rad/s])
