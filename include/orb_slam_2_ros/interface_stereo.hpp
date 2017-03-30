#ifndef ORB_SLAM_2_INTERFACE_STEREO
#define ORB_SLAM_2_INTERFACE_STEREO

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <opencv2/core/core.hpp>

#include <string>

#include "orb_slam_2_ros/interface.hpp"

namespace orb_slam_2_interface {

// The synchronization policy used by the interface to sync stereo images
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image>
    sync_pol;

// Class handling global alignment calculation and publishing
class OrbSlam2InterfaceStereo : public OrbSlam2Interface {
 public:
  // Constructor
  OrbSlam2InterfaceStereo(const ros::NodeHandle& nh,
                          const ros::NodeHandle& nh_private);

 protected:
  // Subscribes to the appropriate ROS topics
  void subscribeToTopics();

  // Callbacks
  void stereoImageCallback(const sensor_msgs::ImageConstPtr& msg_left,
                           const sensor_msgs::ImageConstPtr& msg_right);

  bool stereoRectification();

  // Subscribers
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> left_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> right_sub_;
  std::shared_ptr<message_filters::Synchronizer<sync_pol>> sync_;
  std::string settings_file_path;

  cv::Mat M1l,M2l,M1r,M2r;


};

}  // namespace orb_slam_2_interface

#endif /* ORB_SLAM_2_INTERFACE_STEREO */
