#!/bin/sh

echo "Staring data recording: "

cd /home/j/Dataset/

rosbag record /mv_camera_ros_node/camera_info \
	      /mv_camera_ros_node/image_raw/compressed \
	      /velodyne_points
