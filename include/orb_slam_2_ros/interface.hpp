#ifndef ORB_SLAM_2_INTERFACE
#define ORB_SLAM_2_INTERFACE

#include <memory>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <orb_slam_2/System.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>

#include "orb_slam_2_ros/types.hpp"

namespace orb_slam_2_interface {

// Default values for parameters
static const bool kDefaultVerbose = true;
static const std::string kDefaultFrameId = "world";
static const std::string kDefaultChildFrameId = "cam0";

// Class handling global alignment calculation and publishing
class OrbSlam2Interface {
 public:
  // Constructor
  OrbSlam2Interface(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);

  int image_counter=0;

 protected:
  // Subscribes and Advertises to the appropriate ROS topics
  void advertiseTopics();
  void getParametersFromRos();

  // Callbacks
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  // Publishing functions
  void publishCurrentPose(const Transformation& T,
                          const std_msgs::Header& header);
  void publishCurrentPoseAsTF(const ros::TimerEvent& event);

  void publishCurrentMap(const std::vector<ORB_SLAM2::MapPoint *> &point_cloud,
                         const sensor_msgs::ImageConstPtr& msg_rgb);

    void publishGBArunning(bool isGBArunning);
    void publishLoopClosing(bool loop_closing);
    void publishEssentialGraphOptimization(bool essential_graph_optimization);


  // Helper functions
  void convertOrbSlamPoseToKindr(const cv::Mat& T_cv, Transformation* T_kindr);

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers
  ros::Publisher T_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Timer tf_timer_;
  ros::Publisher Map_pub_;
    ros::Publisher GBA_info_;
    ros::Publisher Loop_closing_info_;
    ros::Publisher Essential_graph_info_;


  // The orb slam system
  std::shared_ptr<ORB_SLAM2::System> slam_system_;

  // The current pose
  Transformation T_W_C_;

  sensor_msgs::PointCloud Map_;

    std_msgs::Bool GBA_running_;
    std_msgs::Bool loop_closing_;
    std_msgs::Bool essential_graph_optimization_;



  // Parameters
  bool verbose_;
  std::string vocabulary_file_path_;
  std::string settings_file_path_;

  // Transform frame names
  std::string frame_id_;
  std::string child_frame_id_;
};

}  // namespace orb_slam_2_interface

#endif /* ORB_SLAM_2_INTERFACE */
