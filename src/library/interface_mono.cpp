#include <cv_bridge/cv_bridge.h>

#include "orb_slam_2_ros/interface_mono.hpp"

namespace orb_slam_2_interface {

OrbSlam2InterfaceMono::OrbSlam2InterfaceMono(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private)
    : OrbSlam2Interface(nh, nh_private) {
  // Getting data and params
  subscribeToTopics();

  // Creating the SlAM system
  slam_system_ = std::shared_ptr<ORB_SLAM2::System>(
      new ORB_SLAM2::System(vocabulary_file_path_, settings_file_path_,
                            ORB_SLAM2::System::MONOCULAR, visualization_));
}

void OrbSlam2InterfaceMono::subscribeToTopics() {
  // Subscribing to the required data topics
  image_sub_ = nh_.subscribe("camera/image_raw", 1,
                             &OrbSlam2InterfaceMono::imageCallback, this);
}

void OrbSlam2InterfaceMono::imageCallback(
    const sensor_msgs::ImageConstPtr& msg) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Handing the image to ORB slam for tracking
  cv::Mat T_C_W_opencv =
      slam_system_->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
  // If tracking successfull
  if (!T_C_W_opencv.empty()) {
    // Converting to kindr transform and publishing
    Transformation T_C_W;
    convertOrbSlamPoseToKindr(T_C_W_opencv, &T_C_W);

    // Saving the transform to the member for publishing as a TF
    if (use_body_transform_) {
      T_W_B_ = T_B_C_ * T_C_W.inverse();
    } else {
      T_W_B_ = T_C_W.inverse();
    }

    // Publish Pose as ROS msg
    publishCurrentPose(T_W_B_, msg->header);
  }
}

}  // namespace orb_slam_2_interface