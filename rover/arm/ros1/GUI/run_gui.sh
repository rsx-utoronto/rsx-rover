#!/bin/bash

rosrun rover qt6_gui.py &
roslaunch rover arm_rviz.launch ik_on:=true
