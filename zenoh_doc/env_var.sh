export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=5   # optional: retries on startup
export ZENOH_SESSION_CONFIG_URI=~/rover_ws/src/rsx-rover/zenoh_doc/zenoh_config.json5