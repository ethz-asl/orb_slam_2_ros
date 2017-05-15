#include <memory>
#include <string>

#include <glog/logging.h>
#include <ros/ros.h>

#include "orb_slam_2_ros/interface.hpp"
#include "orb_slam_2_ros/interface_mono.hpp"
#include "orb_slam_2_ros/interface_stereo.hpp"

// A factory method for creating an interface
std::unique_ptr<orb_slam_2_interface::OrbSlam2Interface> create_interface(
    std::string interface_type, const ros::NodeHandle& nh,
    const ros::NodeHandle& nh_private) {
  // Creating the aligner object subclass dependent on the argument
  std::unique_ptr<orb_slam_2_interface::OrbSlam2Interface> interface;
  if (interface_type == "mono") {
    interface = std::unique_ptr<orb_slam_2_interface::OrbSlam2Interface>(
        new orb_slam_2_interface::OrbSlam2InterfaceMono(nh, nh_private));
  } else if (interface_type == "stereo") {
    interface = std::unique_ptr<orb_slam_2_interface::OrbSlam2Interface>(
        new orb_slam_2_interface::OrbSlam2InterfaceStereo(nh, nh_private));
  } else {
    ROS_FATAL("interface type not recognized. Must be mono or stereo.");
    ros::shutdown();
    exit(1);
  }
  // Returning a pointer to the frame aligner
  return interface;
}

// Standard C++ entry point
int main(int argc, char** argv) {
  // Starting the logging
  google::InitGoogleLogging(argv[0]);
  // Announce this program to the ROS master
  ros::init(argc, argv, "orb_slam_2_ros_node");
  // Creating the node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  // Get the parameter describing the interface type
  static const std::string kDefaultInterfaceType = "mono";
  std::string interface_type = kDefaultInterfaceType;
  nh_private.getParam("interface_type", interface_type);
  // Creating the interface object to do the work
  std::unique_ptr<orb_slam_2_interface::OrbSlam2Interface> interface =
      create_interface(interface_type, nh, nh_private);
  // Spinning
  ros::spin();
  // Exit tranquilly
  return 0;
}