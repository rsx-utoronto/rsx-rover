Protocal for Running Astar Algorithim on Ros2

1. Ensure you have ros2 bag downloaded. The current format of a ros2 bag is a folder with a metadata file and a db3 file. 
2. In terminal, run the rosbag: ros2 bag play mybag/ <-- this is the bag folder
3. Open a new terminal and run the astar script (astar_obstacle_avoidance_algorithim): ros2 run rover astar_obstacle_avoidance_algorithim.py
4. Open a new terminal for rviz2 to visualize the algorithim.