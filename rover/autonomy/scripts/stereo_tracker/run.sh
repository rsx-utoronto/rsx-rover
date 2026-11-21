#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS 2 (supports Humble or Jazzy automatically)
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Fallback to standard FastDDS if rmw_zenoh is configured without an active router
if [ "$RMW_IMPLEMENTATION" = "rmw_zenoh_cpp" ] && ! pgrep -x rmw_zenohd > /dev/null; then
    echo "Zenoh router not running locally; falling back to rmw_fastrtps_cpp for standalone run."
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
fi

# Reset ZED USB device automatically if present (prevents lid-close/sleep UVC locks)
if command -v usbreset >/dev/null 2>&1; then
    sudo -n usbreset 2b03:f880 >/dev/null 2>&1 || true
    sleep 1
fi

exec "$SCRIPT_DIR/.venv/bin/python3" "$SCRIPT_DIR/zed_ros2_node.py" "$@"
