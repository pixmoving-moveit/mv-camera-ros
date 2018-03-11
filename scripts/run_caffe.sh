#!/bin/sh
    
echo "Starting the camera"
./run_camera_sudo.sh &
sleep 10

echo "Staring Lidar"
./run_velodyne.sh