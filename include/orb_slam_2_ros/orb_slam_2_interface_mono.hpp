#ifndef ORB_SLAM_2_INTERFACE_MONO
#define ORB_SLAM_2_INTERFACE_MONO

//#include <memory>
//#include <string>

//#include <geometry_msgs/TransformStamped.h>
//#include <orb_slam_2/System.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
//#include <Eigen/Geometry>
//#include <tf/transform_broadcaster.h>

//#include "orb_slam_2_ros/types.hpp"
#include "orb_slam_2_ros/orb_slam_2_interface.hpp"

namespace orb_slam_2_interface {

/*// Default values for parameters
static const bool kDefaultVerbose = true;
static const std::string kDefaultFrameId = "world";
static const std::string kDefaultChildFrameId = "cam0";*/

// Class handling global alignment calculation and publishing
class OrbSlam2InterfaceMono : public OrbSlam2Interface {
 public:
  // Constructor
  OrbSlam2InterfaceMono(const ros::NodeHandle& nh,
                        const ros::NodeHandle& nh_private);

 protected:
  // Subscribes and Advertises to the appropriate ROS topics
  void subscribeToTopics();
/*  void advertiseTopics();
  void getParametersFromRos();*/

  // Callbacks
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  // Subscribers
  ros::Subscriber image_sub_;

};

}  // namespace orb_slam_2_interface

#endif /* ORB_SLAM_2_INTERFACE_MONO */
