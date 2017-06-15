cmake_minimum_required(VERSION 2.8.0)
project(orb_slam_2_ros)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

OPTION(USE_IMU "Using IMU" OFF)

IF(USE_IMU)
    ADD_DEFINITIONS(-DUSE_IMU)
ENDIF(USE_IMU)

cs_add_library(orb_slam_2_interface
  src/library/interface.cpp
  src/library/interface_mono.cpp
  src/library/interface_stereo.cpp
)

cs_add_executable(orb_slam_2_ros_node
  src/orb_slam_2_ros_node.cpp
)

target_link_libraries(orb_slam_2_ros_node orb_slam_2_interface ${catkin_LIBRARIES})

cs_install()
cs_export()
