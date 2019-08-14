# orb_slam_2_ros

## Overview

This package integrates orb_slam_2 into ROS in what we believe to be a more user friendly way than what is offered by the original library [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)

**Keywords:** ROS, ORB_SLAM2, SLAM

### License

The source code is released under a [GPLv3 license](https://github.com/raulmur/ORB_SLAM2/blob/master/License-gpl.txt) as is the underlying library [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2).

**Author(s): Alex Millane
Maintainer: Alex Millane
Affiliation: Autonomous Systems Lab, ETH Zurich**

The orb_slam_2_ros package has been tested under ROS Indigo on Ubuntu 14.04 and under ROS Kinetic Ubuntu 16.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


## Installation: building from Source

Install [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics).

Make sure you have added your SSH keys to your Github account. For more info check [connecting-to-github-with-ssh](https://help.github.com/articles/connecting-to-github-with-ssh/).

Setup and configure a catkin work space.

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/kinetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel
```
Get this repo

```
cd src
git clone git@github.com:ethz-asl/orb_slam_2_catkin.git
```
Get the deps

```
wstool init
wstool merge orb_slam_2_ros/dependencies.rosinstall
wstool update -j8
```

Build everything

```
catkin build orb_slam_2_ros
```

## Usage

Before launching the node with an example dataset you need to:
 - Download a dataset you can use to run this package from [EUROC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) (for example the one called "Machine Hall 001") (containing rosbag to play and camera calibration);
 - Download and extract the [ORB_SLAM2 vocabulary](https://github.com/raulmur/ORB_SLAM2/blob/master/Vocabulary/ORBvoc.txt.tar.gz).

Run the main node with

    roslaunch orb_slam_2_ros run_orb_slam_2.launch vocabulary_file_path:=<PATH_TO_EXTRACTED_ORB_SLAM2_VOCABULARY>

Please not that this version of orb_slam_2_ros **DOES NOT** rectify images, that is why we are using  [stereo_undistort_node](https://github.com/ethz-asl/image_undistort#stereo_undistort_node).
