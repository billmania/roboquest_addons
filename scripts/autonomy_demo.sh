#!/usr/bin/env bash

cd /usr/src/ros2ws || exit 1
source /opt/ros/humble/setup.bash

source install/setup.bash

ros2 launch roboquest_addons autonomy_demo.launch.py
