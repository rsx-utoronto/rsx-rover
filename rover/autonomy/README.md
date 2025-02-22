# Miscellaneous

## Archived but Possibly Useful Files

A lot of scripts have been moved to archive_files branch to clean the master branch. I think some these might be useful and the following list gives the names of those files: 

- `add_goal.py` - Gives goal to old state machine, converts GPS to UTM 
- `test_add_goal` - Tests add goal
- `aruco_2location_detection 2.py` - Seems like a duplicate
- `aruco_location_detection.py` - Seems like the old version of `aruco_2location_detection.py`
- `convert_gps_to_utm.py` - GPS to UTM conversion code
- `gps_target_waypoint_updater` - publishes GPS target coordinates using input
- look at grid search folder for so many files with different versions of grid search, none of them fully work :(
- `imu_velocity_calculation` - calculates velocity by integrating IMU data
- `listener-truprecision-rsx` - seems like an old file
- grid_map_package folder has files related to implementing grid_map package. We tried to implement that in 2023 for 3D mapping
- `ssid_connect` - CIRC Summer 2023 retrieving the GPS coordinates for night task
- `state_machine` and `state_publisher` - **IMPORTANT** - 2023, 2024 state machine code
- `transform_utm_to_map` - UTM to global map in move_base (starting where the robot is located when move_base is launched)
- `imu_gps.launch` - launch file for inertial-sense GPS (the sensor doesn't work anymore)
- `sensors.launch` - Old file for launching all sensors (probably 2023)
- `STATES_README.md` - has info about 2024 feb state machine (we never ended up completing that)