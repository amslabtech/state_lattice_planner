cmake_minimum_required(VERSION 3.5)
project(state_lattice_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  cmake_modules
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED COMPONENTS system)

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES state_lattice_planner_lib lookup_table_generator_lib trajectory_generator_diff_drive motion_model_diff_drive
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(motion_model_diff_drive
  src/motion_model_diff_drive.cpp
)
add_library(trajectory_generator_diff_drive
  src/trajectory_generator_diff_drive.cpp
)
add_library(lookup_table_generator_lib
  src/lookup_table_generator.cpp
  src/lookup_table_utils.cpp
)
add_library(state_lattice_planner_lib
  src/state_lattice_planner.cpp
  src/lookup_table_utils.cpp
)

add_dependencies(lookup_table_generator_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(state_lattice_planner_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(motion_model_diff_drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(trajectory_generator_diff_drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(state_lattice_planner src/state_lattice_planner_node.cpp src/state_lattice_planner_ros.cpp)
add_executable(lookup_table_generator src/lookup_table_generator_node.cpp)
add_executable(trajectory_viewer src/trajectory_viewer.cpp src/lookup_table_utils.cpp)

add_dependencies(state_lattice_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(lookup_table_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(trajectory_viewer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(state_lattice_planner
  ${catkin_LIBRARIES}
  state_lattice_planner_lib
  trajectory_generator_diff_drive
  motion_model_diff_drive
)
target_link_libraries(lookup_table_generator
  ${catkin_LIBRARIES}
  lookup_table_generator_lib
  trajectory_generator_diff_drive
  motion_model_diff_drive
)
target_link_libraries(trajectory_viewer
  ${catkin_LIBRARIES}
  trajectory_generator_diff_drive
  motion_model_diff_drive
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_subdirectory(test)
endif()
