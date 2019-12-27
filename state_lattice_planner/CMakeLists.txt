cmake_minimum_required(VERSION 2.8.3)
project(state_lattice_planner)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  cmake_modules
  trajectory_generator
)

find_package(Eigen3 REQUIRED COMPONENTS system)
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES state_lattice_planner
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
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

add_executable(state_lattice_planner src/state_lattice_planner_node.cpp)
add_executable(lookup_table_generator src/lookup_table_generator_node.cpp)

add_dependencies(state_lattice_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(lookup_table_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(lookup_table_generator_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(state_lattice_planner_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(state_lattice_planner
  ${catkin_LIBRARIES}
  state_lattice_planner_lib
)
target_link_libraries(lookup_table_generator
  ${catkin_LIBRARIES}
  lookup_table_generator_lib
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(state_lattice_planner_test
        test/state_lattice_planner.test
        test/state_lattice_planner_test.cpp
    )
    target_link_libraries(state_lattice_planner_test
        ${catkin_LIBRARIES}
        lookup_table_generator_lib
        state_lattice_planner_lib
    )
endif()
