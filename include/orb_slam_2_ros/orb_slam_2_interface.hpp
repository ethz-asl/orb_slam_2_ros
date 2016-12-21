#ifndef ORB_SLAM_2_INTERFACE
#define ORB_SLAM_2_INTERFACE

#include <string>
#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <orb_slam_2/System.h>

namespace orb_slam_2_interface {

// Default values for parameters
static const bool kDefaultVerbose = true;

// Class handling global alignment calculation and publishing
class OrbSlam2Interface {
 public:
  // Constructor
  OrbSlam2Interface(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

 protected:
  // Subscribes and Advertises to the appropriate ROS topics
  void subscribeToTopics();
  void advertiseTopics();
  void getParametersFromRos();

  // Callbacks
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers
  ros::Subscriber image_sub_;

  // The orb slam system
  ORB_SLAM2::System* slam_system;

  // Parameters
  bool verbose_;
  std::string vocabulary_file_path_;
  std::string settings_file_path_;

};

}  // namespace orb_slam_2_interface

#endif /* ORB_SLAM_2_INTERFACE */
