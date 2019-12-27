cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generator)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  cmake_modules
)

find_package(Eigen3 REQUIRED COMPONENTS system)
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES trajectory_generator_diff_drive motion_model_diff_drive
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

add_dependencies(motion_model_diff_drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(trajectory_generator_diff_drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(trajectory_generator_test
        test/trajectory_generator.test
        test/trajectory_generator_test.cpp
    )
    target_link_libraries(trajectory_generator_test
        ${catkin_LIBRARIES}
        motion_model_diff_drive
        trajectory_generator_diff_drive
    )
endif()
