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
docker_args="-it"

# Volumes (modify with your own path here)
volumes="-v $PWD/..:/home/$USER/rover_ws/rsx-rover"

# Additional arguments to be able to open GUI
XSOCK=/tmp/.X11-unix
XAUTH=/home/$USER/.Xauthority
other_args="-v $XSOCK:$XSOCK \
    -v $XAUTH:$XAUTH \
    --net=host \
    --privileged \
	-e XAUTHORITY=${XAUTH} \
    -e DISPLAY=$DISPLAY \
    -w /home/$USER/rover_ws/src"


docker run $docker_args \
$volumes \
$other_args \
--name "rsx-dev" \
rsx_dev_rsx \
$command