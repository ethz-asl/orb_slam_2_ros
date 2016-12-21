#include <glog/logging.h>
#include <ros/ros.h>
#include "orb_slam_2_ros/orb_slam_2_interface.hpp"

// Standard C++ entry point
int main(int argc, char** argv) {
  // Starting the logging
  google::InitGoogleLogging(argv[0]);
  // Announce this program to the ROS master
  ros::init(argc, argv, "orb_slam_2_ros_node");
  // Creating the node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  // Creating the object to do the work
  orb_slam_2_interface::OrbSlam2Interface orb_slam_2_interface(nh, nh_private);
  // Spinning
  ros::spin();
  // Exit tranquilly
  return 0;
}
