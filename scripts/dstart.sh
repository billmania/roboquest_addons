#!/usr/bin/env bash

#
# Start the docker container for roboquest_addons
#

IMAGE=$1
NAME=rq_addons

printf "Starting %s on %s\n" "$IMAGE" "$DOCKER_HOST"

docker run -d --rm \
        --privileged \
        --network host \
        --ipc host \
        --env "ROS_DOMAIN_ID=72" \
        --device /dev/ttyUSB0:/dev/ttyUSB0 \
        -v /dev/shm:/dev/shm \
        -v /var/run/dbus:/var/run/dbus \
        -v /run/udev:/run/udev \
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
