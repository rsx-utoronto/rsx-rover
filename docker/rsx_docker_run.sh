#!/bin/bash

# takes 2 positional arguments; first is your ipv4 address (for display), second is path to repo (for bind mount)
# remember to enclose filepath in quotes

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

display="$1:0.0" # Xming display
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
