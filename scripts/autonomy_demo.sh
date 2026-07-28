#!/usr/bin/env bash

cd /usr/src/ros2ws || exit 1
source /opt/ros/humble/setup.bash

source install/setup.bash

#
# Override the navigator.py node's default parameters with ros2 launch. This
# mechanism will NOT override parameters defined in the parameter files
# config/default_nav_params.yaml or persist/nav_params.yaml.
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
