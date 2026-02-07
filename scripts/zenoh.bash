source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# export ZENOH_ROUTER_CONFIG_URI=${HUNTER}/hunter2_zenoh.json5
export ZENOH_ROUTER_CONFIG_URI=~/rover_ws/src/rsx-rover/config/zenoh_rover.json5


if [[ `pidof rmw_zenohd` ]];
then
   echo "Zenoh router already running";
else
   ros2 run rmw_zenoh_cpp rmw_zenohd;
fi
