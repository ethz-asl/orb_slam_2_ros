# orb_slam_2_ros
This package integrates [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) packages with ROS. It also works for some forked versions such as [VIOS](https://github.com/russellaabuchanan/VIOS).


Depends the respective catkin wrapper library:

ORB_SLAM2: https://github.com/ethz-asl/orb_slam_2_catkin

VIOS: https://github.com/russellaabuchanan/orb_slam_2_catkin

When building for VIOS make sure to set the USE_IMU tag, for example:
```
catkin build orb_slam_2_ros -DUSE_IMU=ON -DCMAKE_BUILD_TYPE=Release
```
