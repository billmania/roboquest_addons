#!/usr/bin/env /bin/bash

cd /opt/projects/roboquest/ros2ws

wget -O Dockerfile.roboquest_core https://github.com/billmania/roboquest/raw/refs/heads/main/ros2ws/Dockerfile.roboquest_core
wget -O Dockerfile.roboquest_addons https://github.com/billmania/roboquest/raw/refs/heads/main/ros2ws/Dockerfile.roboquest_addons

cd /opt/projects/roboquest/ros2ws/src
rm -rf roboquest_core rq_msgs roboquest_addons

git clone --branch 116-add-configuration-for-csi-arducam https://github.com/billmania/roboquest_core.git
git clone https://github.com/billmania/rq_msgs.git
git clone https://github.com/billmania/roboquest_addons.git

cd /opt/projects/roboquest/ros2ws
docker system prune -f
docker build -t rq_core_25.1rc3 -f Dockerfile.roboquest_core .
docker build -t rq_addons_4rc2 -f Dockerfile.roboquest_addons .

exit 0
