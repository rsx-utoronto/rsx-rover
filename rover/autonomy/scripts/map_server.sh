#! /bin/bash

sleep 6

while true; do

    rosrun map_server map_server ~/rsx_ws/src/rsx-rover/rover/autonomy/maps/current_map.yaml frame_id:=map /map:=/cur_oc /map_metadata:=cur_oc_metadata
    echo done!
    sleep 5

done

