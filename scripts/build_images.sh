#!/usr/bin/env /bin/bash

VERSION=2

mkdir -p /opt/projects/roboquest/ros2ws
cd /opt/projects/roboquest/ros2ws

wget -O Dockerfile.roboquest_addons \
    https://github.com/billmania/roboquest/raw/refs/heads/main/ros2ws/Dockerfile.roboquest_addons

cd /opt/projects/roboquest/ros2ws/src
rm -rf roboquest_core rq_msgs roboquest_addons

git clone \
    https://github.com/billmania/roboquest_addons.git

cd /opt/projects/roboquest/ros2ws
docker system prune -f
docker build -t rq_addons_4rc2 -f Dockerfile.roboquest_addons .

exit 0
