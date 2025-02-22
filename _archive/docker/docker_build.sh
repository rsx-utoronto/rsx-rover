#!/bin/bash

username="rsx"
userid=$UID

echo $username
echo $userid

echo ""
echo "Building image rsx_dev"
echo ""

docker image build --build-arg username0=$username \
--build-arg userid0=$userid \
--shm-size=64g -t \
rsx_dev_$username . 