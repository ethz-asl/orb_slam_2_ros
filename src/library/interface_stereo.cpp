#include "orb_slam_2_ros/interface_stereo.hpp"

namespace orb_slam_2_interface {

OrbSlam2InterfaceStereo::OrbSlam2InterfaceStereo(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : OrbSlam2Interface(nh, nh_private) {
  // Getting data and params
  subscribeToTopics();
  // advertiseTopics();
  // getParametersFromRos();
  slam_system_ = std::shared_ptr<ORB_SLAM2::System>(
      new ORB_SLAM2::System(vocabulary_file_path_, settings_file_path_,
                            ORB_SLAM2::System::STEREO, true));
  // Starting thread to publish loop closure trajectories
  mpt_loop_closure_publisher =
      new thread(&OrbSlam2InterfaceStereo::runPublishUpdatedTrajectory, this);
}

void OrbSlam2InterfaceStereo::subscribeToTopics() {
  // Subscribing to the stereo images
  left_sub_ = std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
      new message_filters::Subscriber<sensor_msgs::Image>(
          nh_, "camera/left/image_raw", 1));
  right_sub_ = std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
      new message_filters::Subscriber<sensor_msgs::Image>(
          nh_, "camera/right/image_raw", 1));
  // Creating a synchronizer
  sync_ = std::shared_ptr<message_filters::Synchronizer<sync_pol>>(
      new message_filters::Synchronizer<sync_pol>(sync_pol(10), *left_sub_,
                                                  *right_sub_));
  // Registering the synchronized image callback
  sync_->registerCallback(
      boost::bind(&OrbSlam2InterfaceStereo::stereoImageCallback, this, _1, _2));
}

void OrbSlam2InterfaceStereo::stereoImageCallback(
    const sensor_msgs::ImageConstPtr& msg_left,
    const sensor_msgs::ImageConstPtr& msg_right) {
  // Converting to OpenCV frames
  cv_bridge::CvImageConstPtr cv_ptr_left;
  cv_bridge::CvImageConstPtr cv_ptr_right;
  convertFrames(msg_left, msg_right, cv_ptr_left, cv_ptr_right);

  // Performing tracking using the frames
  performTracking(cv_ptr_left, cv_ptr_right);
  // Publishing results
  publishCurrentPose(T_W_C_, msg_left->header);
}

// Performs tracking given the current frames
void OrbSlam2InterfaceStereo::performTracking(
    const cv_bridge::CvImageConstPtr cv_ptr_left,
    const cv_bridge::CvImageConstPtr cv_ptr_right) {
  // Handing the image to ORB slam for tracking
  cv::Mat T_C_W_opencv =
      slam_system_->TrackStereo(cv_ptr_left->image, cv_ptr_right->image,
                                cv_ptr_left->header.stamp.toSec());
  // If tracking successfull
  if (!T_C_W_opencv.empty()) {
    // Converting to kindr transform and publishing
    Transformation T_C_W, T_W_C;
    convertOrbSlamPoseToKindr(T_C_W_opencv, &T_C_W);
    T_W_C = T_C_W.inverse();
    // Saving the transform to the member for publishing as a TF
    T_W_C_ = T_W_C;
  }
}


void OrbSlam2InterfaceStereo::convertFrames(
    const sensor_msgs::ImageConstPtr& msg_left,
    const sensor_msgs::ImageConstPtr& msg_right,
    cv_bridge::CvImageConstPtr& cv_ptr_left,
    cv_bridge::CvImageConstPtr& cv_ptr_right) {
  // Copy the ros image message to cv::Mat.
  try {
    cv_ptr_left = cv_bridge::toCvShare(msg_left);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  try {
    cv_ptr_right = cv_bridge::toCvShare(msg_right);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

}  // namespace orb_slam_2_interface