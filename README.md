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

### Dependencies
Make sure you have added your SSH keys to your Github account. For more info check [connecting-to-github-with-ssh](https://help.github.com/articles/connecting-to-github-with-ssh/).

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [catkin_simple](https://github.com/catkin/catkin_simple)
	```
	cd ~/catkin_workspace/src
	git clone git@github.com:catkin/catkin_simple.git # cloning over SSH (github account needed)
	# git clone https://github.com/catkin/catkin_simple.git # cloning over HTTPS (no github account needed)
	```
- [catkinized version of the orb_slam_2](https://github.com/ethz-asl/orb_slam_2_catkin)
	```
	cd ~/catkin_workspace/src
	git clone git@github.com:ethz-asl/orb_slam_2_catkin.git # cloning over SSH (github account needed)
	# git clone https://github.com/ethz-asl/orb_slam_2_catkin.git # cloning over HTTPS (no github account needed)
	```
- [image_undistort](https://github.com/ethz-asl/image_undistort)
	```
	cd ~/catkin_workspace/src
	git clone git@github.com:ethz-asl/image_undistort.git # cloning over SSH (github account needed)
	# git clone https://github.com/ethz-asl/image_undistort.git # cloning over HTTPS (no github account needed) 
	```
- [pangolin_catkin](https://github.com/ethz-asl/pangolin_catkin)
	```
	cd ~/catkin_workspace/src
	git clone git@github.com:uzh-rpg/pangolin_catkin.git # cloning over SSH (github account needed)
	# git clone https://github.com/ethz-asl/pangolin_catkin.git # cloning over HTTPS (no github account needed) 
	```

### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd ~/catkin_workspace/src
	git clone git@github.com:ethz-asl/orb_slam_2_ros.git # cloning over SSH (github account needed)
    # git clone https://github.com/ethz-asl/orb_slam_2_ros.git # cloning over HTTPS (no github account needed)
	cd ../
	catkin build orb_slam_2_ros


By building the higher level package orb_slam_2_ros you will even compile its dependencies. This includes the compilation of orb_slam_2_catkin, which will **automatically** install the original library ORB_SLAM2 in your system, meaning you **do not need** to do anything else to then use ORB_SLAM2.

## Usage

**PART UNDER REVIEW! DO NOT USE THE FOLLOWING CODE AS IT DOES NOT WORK YET!**


~~Before launching the node with an example dataset you need to:~~
 - ~~Download a dataset you can use to run this package from [EUROC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) (for example the one called "Machine Hall 001") (containing rosbag to play and camera calibration)~~;
 - ~~Download and extract the [ORB_SLAM2 vocabulary](https://github.com/raulmur/ORB_SLAM2/blob/master/Vocabulary/ORBvoc.txt.tar.gz)~~.

~~Run the main node with~~

    roslaunch orb_slam_2_ros run_orb_slam_2.launch vocabulary_file_path:=<PATH_TO_EXTRACTED_ORB_SLAM2_VOCABULARY>

~~Please not that this version of orb_slam_2_ros **DOES NOT** rectify images, that is why we are using  [stereo_undistort_node](https://github.com/ethz-asl/image_undistort#stereo_undistort_node)~~.
