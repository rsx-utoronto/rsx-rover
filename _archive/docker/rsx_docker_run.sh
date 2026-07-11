#!/bin/bash

########
# Init #
########

echo ""
echo "Starting up container from rsx_dev_rsx"
echo ""

##########################
# Start docker container #
##########################

# Docker run arguments
docker_args="-it --rm"

display="$1:0.0" # X display
repo_path="$2"   # bind mount (see difference between bind mounts and docker volumes: https://www.dltlabs.com/blog/bind-mounts-volumes-indocker-133067)

other_args="
    --privileged \
    --net=host \
    -e DISPLAY=$display
    -v $repo_path:/home/rsx/rover_ws/src/rsx-rover:rw"

winpty docker run \
    $docker_args \
    $other_args \
    --name "rsx-dev" \
    rsx_dev_rsx
