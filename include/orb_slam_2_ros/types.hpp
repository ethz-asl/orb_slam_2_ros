#ifndef ORB_SLAM_2_TYPES
#define ORB_SLAM_2_TYPES

#include <kindr/minimal/quat-transformation.h>
#include <kindr/minimal/rotation-quaternion.h>

namespace orb_slam_2_interface {

// Convenience typedef
typedef kindr::minimal::QuatTransformation Transformation;
typedef kindr::minimal::RotationQuaternion Quaternion;

}  // namespace orb_slam_2_interface

#endif /* ORB_SLAM_2_TYPES */
