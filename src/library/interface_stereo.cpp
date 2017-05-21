#include <cv_bridge/cv_bridge.h>

#include "orb_slam_2_ros/interface_stereo.hpp"

namespace orb_slam_2_interface {

OrbSlam2InterfaceStereo::OrbSlam2InterfaceStereo(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : OrbSlam2Interface(nh, nh_private) {
  // Initialize rectification state to false
  stereo_rectified_ = false;
  // Getting data and params
  subscribeToTopics();
  // Getting stereo rectification maps
  stereoRectification();
  slam_system_ = std::shared_ptr<ORB_SLAM2::System>(
      new ORB_SLAM2::System(vocabulary_file_path_, settings_file_path_,
                            ORB_SLAM2::System::STEREO, visualization_));
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


  imu_sub_ = nh_.subscribe("/imu0",10,&OrbSlam2InterfaceStereo::ImuCallback, this);
}

void OrbSlam2InterfaceStereo::stereoRectification() {
  // Load settings related to stereo calibration
  cv::FileStorage fsSettings(settings_file_path_, cv::FileStorage::READ);

  cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r, T_R_L, Q_;
  fsSettings["LEFT.K"] >> K_l;
  fsSettings["RIGHT.K"] >> K_r;

  fsSettings["LEFT.D"] >> D_l;
  fsSettings["RIGHT.D"] >> D_r;

  int rows_l = fsSettings["LEFT.height"];
  int cols_l = fsSettings["LEFT.width"];
  int rows_r = fsSettings["RIGHT.height"];
  int cols_r = fsSettings["RIGHT.width"];

  if (K_l.empty() || K_r.empty() || D_l.empty() || D_r.empty() || rows_l == 0 ||
      rows_r == 0 || cols_l == 0 || cols_r == 0) {
    cerr << "ERROR: Distortion parameters for stereo rectification are missing!"
         << endl;
    return;
  }

  fsSettings["LEFT.P"] >> P_l;
  fsSettings["RIGHT.P"] >> P_r;

  fsSettings["LEFT.R"] >> R_l;
  fsSettings["RIGHT.R"] >> R_r;

  fsSettings["T_RIGHT_LEFT"] >> T_R_L;

  if (P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty()) {
    if (!T_R_L.empty()) {
      cerr << "WARNING: Rectification matrices are missing. Calculating now."
           << endl;
      cv::stereoRectify(K_l, D_l, K_r, D_r, cv::Size(cols_l, rows_l),
                        T_R_L.rowRange(0, 3).colRange(0, 3),
                        T_R_L.col(3).rowRange(0, 3), R_l, R_r, P_l, P_r, Q_);
    }
    cerr << "ERROR: Stereo extrinsic transform missing!" << endl;
    return;
  }

  cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3),
                              cv::Size(cols_l, rows_l), CV_32F, left_rectification_map1_,
                              left_rectification_map2_);
  cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3),
                              cv::Size(cols_r, rows_r), CV_32F, right_rectification_map1_,
                              right_rectification_map2_);

  stereo_rectified_ = true;

  return;
}

void OrbSlam2InterfaceStereo::stereoImageCallback(
    const sensor_msgs::ImageConstPtr& msg_left,
    const sensor_msgs::ImageConstPtr& msg_right) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr_left;
  cv::Mat T_W_C_opencv;

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

  if (stereo_rectified_) {
    cv::Mat imLeft, imRight;
    cv::remap(cv_ptr_left->image, imLeft, left_rectification_map1_, left_rectification_map2_, cv::INTER_LINEAR);
    cv::remap(cv_ptr_right->image, imRight, right_rectification_map1_, right_rectification_map2_, cv::INTER_LINEAR);
    // Handing the image to ORB slam for tracking
    T_W_C_opencv = slam_system_->TrackStereo(imLeft, imRight,
                                             cv_ptr_left->header.stamp.toSec());
  } else {
    T_W_C_opencv =
        slam_system_->TrackStereo(cv_ptr_left->image, cv_ptr_right->image,
                                  cv_ptr_left->header.stamp.toSec());
  }

  // If tracking successfull
  if (!T_W_C_opencv.empty()) {
    // Converting to kindr transform and publishing
    Transformation T_W_C;
    convertOrbSlamPoseToKindr(T_W_C_opencv, &T_W_C);

    // Saving the transform to the member for publishing as a TF
    if (use_body_transform_) {
      T_W_B_ = T_V_B_ * T_B_C_ * T_W_C.inverse();
    } else {
      T_W_B_ = T_W_C.inverse();
    }

    publishCurrentPose(T_W_B_, msg_left->header);
  }
}

void OrbSlam2InterfaceStereo::ImuCallback(const sensor_msgs::Imu& imu) {
// cout << "Got IMU data" << endl;

#ifdef USING_IMU
  struct ORB_SLAM2::IMUMeas newMeas;
  newMeas.ang_vel = Eigen::Vector3d::Zero(3);
  newMeas.lin_acc = Eigen::Vector3d::Zero(3);

  ORB_SLAM2::MotionModel* mpMotionModel = slam_system_->GetMotionModeler();

  uint64_t seconds = imu.header.stamp.sec;
  uint64_t nseconds = imu.header.stamp.nsec;

  seconds = seconds % 100000;

  newMeas.time_stamp = seconds * 1000 + nseconds / 1000000;

  newMeas.ang_vel(0) = imu.angular_velocity.x;
  newMeas.ang_vel(1) = imu.angular_velocity.y;
  newMeas.ang_vel(2) = imu.angular_velocity.z;

  newMeas.lin_acc(0) = imu.linear_acceleration.x;
  newMeas.lin_acc(1) = imu.linear_acceleration.y;
  newMeas.lin_acc(2) = imu.linear_acceleration.z;

  mpMotionModel->NewMeasurement(&newMeas);
  mpMotionModel->ProcessIMUMeas();
#endif
}

}  // namespace orb_slam_2_interface
