#ifndef ORB_SLAM_2_INTERFACE
#define ORB_SLAM_2_INTERFACE

#include <string>
#include <memory>
//#include <vector>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>

#include <orb_slam_2/System.h>

/*#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
*/

/*#include <Eigen/Geometry>

orb_slam_2_catkin

#include <eigen_conversions/eigen_msg.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <kindr/minimal/quat-transformation.h>
#include <tf/transform_broadcaster.h>*/

namespace orb_slam_2_interface {

// Default values for parameters
static const bool kDefaultVerbose = true;

/*// Convenience typedef
typedef kindr::minimal::QuatTransformation Transformation;

struct TransformationStamped {
  ros::Time stamp;
  Transformation transformation;
};*/

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

  // Publishes the resulting transform
  //void publishTFTransform(const ros::TimerEvent& event);

  // Callbacks
  // Subscribing to the transforms
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers
  ros::Subscriber image_sub_;

  // The orb slam system
  ORB_SLAM2::System* slam_system;


/*
  // TF publishing timer and broadcaster object.
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Timer tf_timer_;

  // Stores the position of the current hoop in the world frame
  TransformationStamped transform_;*/

  // Parameters
  bool verbose_;
  std::string vocabulary_file_path_;
  std::string settings_file_path_;
  /*
  std::string local_frame_name_;
  std::string global_frame_name_;*/
};

}  // namespace orb_slam_2_interface

#endif /* ORB_SLAM_2_INTERFACE */
