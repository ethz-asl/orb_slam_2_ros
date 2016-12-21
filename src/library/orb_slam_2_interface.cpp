#include "orb_slam_2_ros/orb_slam_2_interface.hpp"

#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>

namespace orb_slam_2_interface {

OrbSlam2Interface::OrbSlam2Interface(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), verbose_(kDefaultVerbose) {
  // Getting data and params
  subscribeToTopics();
  advertiseTopics();
  getParametersFromRos();
  // Creating the SlAM system
  slam_system =
      new ORB_SLAM2::System(vocabulary_file_path_, settings_file_path_,
                            ORB_SLAM2::System::MONOCULAR, true);
}

void OrbSlam2Interface::subscribeToTopics() {
  // Subscribing to the required data topics
  image_sub_ = nh_.subscribe("camera/image_raw", 1,
                             &OrbSlam2Interface::imageCallback, this);
}

void OrbSlam2Interface::advertiseTopics() {
  // Advertising topics
}

void OrbSlam2Interface::getParametersFromRos() {
  // Getting the paths to the files required by orb slam
  CHECK(nh_private_.getParam("vocabulary_file_path", vocabulary_file_path_))
      << "Please provide the vocabulary_file_path as a ros param.";
  CHECK(nh_private_.getParam("settings_file_path", settings_file_path_))
      << "Please provide the settings_file_path as a ros param.";
}

void OrbSlam2Interface::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // Handing the image to ORB slam for tracking
  slam_system->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
}

}  // namespace orb_slam_2_interface
