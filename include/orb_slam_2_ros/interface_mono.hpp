#ifndef ORB_SLAM_2_INTERFACE_MONO
#define ORB_SLAM_2_INTERFACE_MONO

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "orb_slam_2_ros/interface.hpp"

namespace orb_slam_2_interface {

// Class handling global alignment calculation and publishing
class OrbSlam2InterfaceMono : public OrbSlam2Interface {
 public:
  // Constructor
  OrbSlam2InterfaceMono(const ros::NodeHandle& nh,
                        const ros::NodeHandle& nh_private);

 protected:
  // Subscribes to the appropriate ROS topics
  void subscribeToTopics();

  // Callbacks
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  // Subscribers
  ros::Subscriber image_sub_;
};

}  // namespace orb_slam_2_interface

#endif /* ORB_SLAM_2_INTERFACE_MONO */
