#!/usr/bin/env bash

#
# Start the docker container for roboquest_addons
#

IMAGE=$1
NAME="rq_addons"
PERSIST_DIR="/usr/src/ros2ws/install/roboquest_addons/share/roboquest_addons/persist"
RPLIDAR_DEVICE="/dev/ttyUSB0"

printf "Starting %s on %s\n" "$IMAGE" "$(docker context show)"

docker run -it --rm \
        --privileged \
        --network host \
        --ipc host \
        --env "ROS_DOMAIN_ID=72" \
        --device /dev/i2c-6:/dev/i2c-6 \
        --device ${RPLIDAR_DEVICE}:/dev/ttyUSB0 \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /dev/shm:/dev/shm \
        -v /var/run/dbus:/var/run/dbus \
        -v /run/udev:/run/udev \
        -v /opt/persist:${PERSIST_DIR} \
        -v ros_logs:/root/.ros/log \
        --name $NAME \
        "$IMAGE"

if [[ $? = 0 ]]
then
    docker container ls
    printf "\nStarted\n"
else
    printf "Failed to start %s\n" $NAME
fi

exit 0
