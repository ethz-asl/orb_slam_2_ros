#ifndef ORB_SLAM_2_INTERFACE
#define ORB_SLAM_2_INTERFACE

#include <memory>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <kindr/minimal/quat-transformation.h>
#include <kindr/minimal/rotation-quaternion.h>
#include <orb_slam_2/System.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Geometry>

namespace orb_slam_2_interface {

// Convenience typedef
typedef kindr::minimal::QuatTransformation Transformation;
typedef kindr::minimal::RotationQuaternion Quaternion;

// NOT NEEDED FOR NOW.
/*// Stamped transform type
struct TransformationStamped {
  ros::Time stamp;
  Transformation transformation;
};
*/
// Default values for parameters
static const bool kDefaultVerbose = true;
static const std::string kDefaultChildFrameName = "world";

// Class handling global alignment calculation and publishing
class OrbSlam2Interface {
 public:
  // Constructor
  OrbSlam2Interface(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);

 protected:
  // Subscribes and Advertises to the appropriate ROS topics
  void subscribeToTopics();
  void advertiseTopics();
  void getParametersFromRos();

  // Callbacks
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  // Publishing functions
  void publishCurrentPose(const Transformation& T, const std_msgs::Header& header);

  // Helper functions
  void convertOrbSlamPoseToKindr(const cv::Mat& T_cv, Transformation* T_kindr);

  // DEBUG
  // TODO(alexmillane): Remove this in a release.
  std::string type2str(int type);

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers
  ros::Subscriber image_sub_;

  // Publishers
  ros::Publisher T_pub_;

  // The orb slam system
  std::shared_ptr<ORB_SLAM2::System> slam_system_;

  // Parameters
  bool verbose_;
  std::string vocabulary_file_path_;
  std::string settings_file_path_;

  // Transform frame names
  std::string local_frame_name_;

};

}  // namespace orb_slam_2_interface

#endif /* ORB_SLAM_2_INTERFACE */
