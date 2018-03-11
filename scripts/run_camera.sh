#!/bin/sh
. /opt/ros/indigo/setup.sh
export ROS_MASTER_URI=http://localhost:11311
. /home/j/Move-It/mv-camera-ws/devel/setup.sh
roslaunch mv_camera_ros mv-camera-ros.launch
