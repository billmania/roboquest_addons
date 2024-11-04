#!/usr/bin/env bash

cd /usr/src/ros2ws
source /opt/ros/humble/setup.bash

source install/setup.bash

ros2 launch rplidar_ros rplidar_c1_launch.py
