#!/usr/bin/env bash

cd /usr/src/ros2ws || exit 1
source /opt/ros/humble/setup.bash

source install/setup.bash

#
# Define parameters with ros2 launch
#
# ros2 launch roboquest_addons autonomy_demo.launch.py max_search_time:=20.0 move_speed:=0.2
#
# Change parameters with ros2 param set. All of the command line
# arguments are strings, so apostrophes are needed only to embed
# SPACEs or characters special to the shell. The parameter's node
# handles casting the new value to the appropriate type.
#
# ros2 param set /navigator turn_speed 2.0
#
ros2 launch roboquest_addons autonomy_demo.launch.py
