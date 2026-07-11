#! /bin/bash

sleep 2

while true; do

    rosrun map_server map_saver -f ~/rsx_ws/src/rsx-rover/rover/autonomy/maps/current_map map:=/ransac_grid
    rosrun map_server map_server ~/rsx_ws/src/rsx-rover/rover/autonomy/maps/current_map.yaml frame_id:=map /map:=/cur_oc /map_metadata:=/cur_oc_metadata anonymous:=false
    # rosnode kill map_server
    echo done!
    sleep 5
    

done

