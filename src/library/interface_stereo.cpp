#include <cv_bridge/cv_bridge.h>

#include "orb_slam_2_ros/interface_stereo.hpp"

namespace orb_slam_2_interface {

OrbSlam2InterfaceStereo::OrbSlam2InterfaceStereo(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private)
    : OrbSlam2Interface(nh, nh_private) {

  nh_private.getParam("settings_file_path", settings_file_path);
  // Getting data and params
  subscribeToTopics();
  advertiseTopics();
  //getParametersFromRos();
  slam_system_ = std::shared_ptr<ORB_SLAM2::System>(
      new ORB_SLAM2::System(vocabulary_file_path_, settings_file_path_,
                            ORB_SLAM2::System::STEREO, true));
}

void OrbSlam2InterfaceStereo::subscribeToTopics() {
  // Subscribing to the stereo images
  left_sub_ = std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
      new message_filters::Subscriber<sensor_msgs::Image>(
          nh_, "cam0/image_raw", 1));
  right_sub_ = std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
      new message_filters::Subscriber<sensor_msgs::Image>(
          nh_, "cam1/image_raw", 1));
  // Creating a synchronizer
  sync_ = std::shared_ptr<message_filters::Synchronizer<sync_pol>>(
      new message_filters::Synchronizer<sync_pol>(sync_pol(10), *left_sub_,
                                                  *right_sub_));
  // Registering the synchronized image callback
  sync_->registerCallback(
      boost::bind(&OrbSlam2InterfaceStereo::stereoImageCallback, this, _1, _2))

}

bool OrbSlam2InterfaceStereo::stereoRectification()
{
    // Load settings related to stereo calibration
    cv::FileStorage fsSettings(settings_file_path, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        ROS_ERROR("ERROR: Wrong path to settings");
        return false;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r, T_c0c1, Q_;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    fsSettings["T_c0_c1"] >> T_c0c1;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        ROS_ERROR("ERROR: Calibration parameters to rectify stereo are missing!");
        return false;
    }

    cv::stereoRectify(K_l,D_l,K_r,D_r,cv::Size(cols_l,rows_l),T_c0c1.rowRange(0,3).colRange(0,3),T_c0c1.col(3).rowRange(0,3),
    R_l,R_r,P_l,P_r,Q_);

    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

    return true;
}

void OrbSlam2InterfaceStereo::stereoImageCallback(
    const sensor_msgs::ImageConstPtr& msg_left,
    const sensor_msgs::ImageConstPtr& msg_right) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr_left;
  try {
    cv_ptr_left = cv_bridge::toCvShare(msg_left);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv_bridge::CvImageConstPtr cv_ptr_right;
  try {
    cv_ptr_right = cv_bridge::toCvShare(msg_right);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat imLeft, imRight;
  cv::remap(cv_ptr_left->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
  cv::remap(cv_ptr_right->image,imRight,M1r,M2r,cv::INTER_LINEAR);
  // Handing the image to ORB slam for tracking
  cv::Mat T_C_W_opencv =
      slam_system_->TrackStereo(imLeft, imRight,cv_ptr_left->header.stamp.toSec());
  // If tracking successfull
  if (!T_C_W_opencv.empty()) {
    // Converting to kindr transform and publishing
    Transformation T_C_W, T_W_C;
    convertOrbSlamPoseToKindr(T_C_W_opencv, &T_C_W);
    T_W_C = T_C_W.inverse();
    publishCurrentPose(T_W_C, msg_left->header);
    // Saving the transform to the member for publishing as a TF
    T_W_C_ = T_W_C;
  }

}

}  // namespace orb_slam_2_interface