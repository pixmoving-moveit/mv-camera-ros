#!/bin/sh
. /opt/ros/kinetic/setup.sh
export ROS_MASTER_URI=http://localhost:11311
. /home/j/hardware_support/mv-camera-ros_ws/devel/setup.sh
roslaunch mv_camera_ros mv-camera-ros.launch
