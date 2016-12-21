#include "orb_slam_2_ros/orb_slam_2_interface.hpp"

#include <opencv2/core/core.hpp>

#include <cv_bridge/cv_bridge.h>

namespace orb_slam_2_interface {

OrbSlam2Interface::OrbSlam2Interface(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), verbose_(kDefaultVerbose) {
  // Getting data and params
  subscribeToTopics();
  advertiseTopics();
  getParametersFromRos();
  // Creating the SlAM system
  slam_system = new ORB_SLAM2::System(vocabulary_file_path_, settings_file_path_, ORB_SLAM2::System::MONOCULAR, true);
}

void OrbSlam2Interface::subscribeToTopics() {
  // Subscribing to the required data topics
  image_sub_ = nh_.subscribe("camera/image_raw", 1,
                                 &OrbSlam2Interface::imageCallback, this);
}

void OrbSlam2Interface::advertiseTopics() {
// Advertising topics
/*  tf_timer_ = nh_.createTimer(ros::Duration(0.01),
                              &OrbSlam2Interface::publishTFTransform, this);
*/}

void OrbSlam2Interface::getParametersFromRos() {

  // TODO(alexmillane): make these required! GLOG
  nh_private_.getParam("vocabulary_file_path", vocabulary_file_path_);
  nh_private_.getParam("settings_file_path", settings_file_path_);

  // DEBUG
  std::cout << "vocabulary_file_path: " << vocabulary_file_path_ << std::endl;
  std::cout << "settings_file_path: " << settings_file_path_ << std::endl;

}

void OrbSlam2Interface::imageCallback(const sensor_msgs::ImageConstPtr& msg) {

  // DEBUG
  std::cout << "Image recieved" << std::endl;

  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
      cv_ptr = cv_bridge::toCvShare(msg);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  // Handing the image to ORB slam for tracking
  slam_system->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
}

/*void OrbSlam2Interface::publishTFTransform(const ros::TimerEvent& event) {
  tf::Transform tf_transform;
  tf::transformKindrToTF(transform_.transformation, &tf_transform);
  tf_broadcaster_.sendTransform(tf::StampedTransform(
      tf_transform, ros::Time::now(), global_frame_name_, local_frame_name_));
}*/

}  // namespace orb_slam_2_interface