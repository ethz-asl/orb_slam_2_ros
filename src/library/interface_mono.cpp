#include <cv_bridge/cv_bridge.h>

#include "orb_slam_2_ros/interface_mono.hpp"

namespace orb_slam_2_interface {

OrbSlam2InterfaceMono::OrbSlam2InterfaceMono(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private)
    : OrbSlam2Interface(nh, nh_private) {
  // Getting data and params
  subscribeToTopics();

  if(use_body_transform_){
    if(!getBodyTransform()){
      ros::shutdown();
      exit(1);
    }
  }

  // Creating the SlAM system
  slam_system_ = std::shared_ptr<ORB_SLAM2::System>(
      new ORB_SLAM2::System(vocabulary_file_path_, settings_file_path_,
                            ORB_SLAM2::System::MONOCULAR, false));
}

void OrbSlam2InterfaceMono::subscribeToTopics() {
  // Subscribing to the required data topics
  image_sub_ = nh_.subscribe("cam0/image_raw", 1,
                             &OrbSlam2InterfaceMono::imageCallback, this);
}

bool OrbSlam2InterfaceMono::getBodyTransform()
{

  cv::FileStorage fsSettings(settings_file_path_, cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
      ROS_ERROR("ERROR: Wrong path to settings");
      return false;
  }

  cv::Mat T_C0_B;
  Transformation T_C_B;

  fsSettings["T_c0_fcuimu"] >> T_C0_B;
  convertOrbSlamPoseToKindr(T_C0_B, &T_C_B);
  T_B_C_ = T_C_B.inverse();

  return true;
}

void OrbSlam2InterfaceMono::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  //TODO Add distortion corredtion for Mono
  // Handing the image to ORB slam for tracking
  cv::Mat T_C_W_opencv =
      slam_system_->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
  // If tracking successfull
  if (!T_C_W_opencv.empty()) {
    // Converting to kindr transform and publishing
    Transformation T_C_W, T_output;
    convertOrbSlamPoseToKindr(T_C_W_opencv, &T_C_W);

    if(use_body_transform_)
    {
      T_output = T_B_C_*T_C_W.inverse();
    }
    else
    {
     T_output = T_C_W.inverse();
    }
    
    publishCurrentPose(T_output, msg->header);

    // Saving the transform to the member for publishing as a TF
    T_W_B_ = T_output;
  }
}

}  // namespace orb_slam_2_interface