#!/bin/bash

########
# Init #
########

echo ""
echo "Running rsx dev docker"
echo ""

command=""


##########################
# Start docker container #
##########################

# Docker run arguments
docker_args="-it --rm"
username="cathe"

# Volumes (modify with your own path here)
volumes="-v /c/Users/$username/rsx-rover:/home/rsx/rover_ws/src/rsx-rover:rw"

echo $PWD

other_args="
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY "

winpty docker run $docker_args \
$other_args \
-e DISPLAY=$DISPLAY \
--name "rsx-dev" \
rsx_dev_rsx \
$command
