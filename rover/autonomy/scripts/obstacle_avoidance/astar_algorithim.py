#!/usr/bin/env python3

"""
A* path planning with OctoMap integration for obstacle avoidance.

Want to get map of surroundngs using octomap within 5m -> to process it we will give each block a value representing its cost. 
The cost depends on height, density -> we go to lowest cost.
Cost depends on height. follow threshold.
1 for height
1 for density 
"""

import rospy
import numpy as np
from octomap_msgs.msg import Octomap
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from queue import PriorityQueue

class OctoMapAStar:
    def __init__(self):
        rospy.init_node("octomap_a_star_planner", anonymous=True)

        # Parameters
        self.map_topic = rospy.get_param("~map_topic", "/octomap_binary")
        self.waypoint_topic = rospy.get_param("~waypoint_topic", "/waypoint")
        self.update_rate = rospy.get_param("~update_rate", 1.0)  # Frequency in Hz
        self.height_min = rospy.get_param("~height_min", 0.2)  # Min height for obstacles
        self.height_max = rospy.get_param("~height_max", 15)  # Max height for obstacles

        # Publishers and Subscribers
        self.map_sub = rospy.Subscriber(self.map_topic, Octomap, self.octomap_callback)
        self.waypoint_pub = rospy.Publisher(self.waypoint_topic, PoseStamped, queue_size=10)

        # Map and planning variables
        self.occupancy_grid = None
        self.grid_resolution = 0.1  # Resolution of 2D grid (meters per cell)
        self.grid_origin = None
        self.goal = None
        self.rate = rospy.Rate(self.update_rate)

    def octomap_callback(self, msg):
        """
        Convert OctoMap to a 2D occupancy grid.
        """
        # Parse OctoMap data and project into 2D grid
        octomap_data = self.process_octomap(msg) # extract 3d data from octomap and converts into 2d grid 
        if octomap_data is not None:
            self.occupancy_grid = octomap_data 
            rospy.loginfo("2D occupancy grid generated from OctoMap.")
    
    def process_octomap(self, octomap_msg):
        """
        Process OctoMap into a 2D occupancy grid based on actual height values.
        """
        # Example grid size and resolution (adjust as needed)
        grid_size = (100, 100)  # Number of cells in x and y
        resolution = 0.1        # Grid resolution in meters (each cell represents 10cm)

        # initialize an empty grid -> values are all 0 at first
        occupancy_grid = np.zeros(grid_size, dtype=np.int8) 

        # Parse the OctoMap message to extract 3D occupancy data
        # (You may need to use a library like `pyoctomap` to parse the data)
        occupied_points = self.decode_octomap(octomap_msg)  # Returns a list of (x, y, z) points

        # Iterate through the occupied points
        for point in occupied_points:
            x, y, z = point

            # Convert the 3D point's x, y coordinates into grid cell indices
            grid_x = int(x / resolution)
            grid_y = int(y / resolution)

            # Ensure the indices are within the grid bounds
            if 0 <= grid_x < grid_size[0] and 0 <= grid_y < grid_size[1]:
                # Check if height (z) is in the occupied range
                if 200 <= z <= 1500: # threshold for occupied or not 0.2-1.5m 
                    # Assign a cost based on height
                    # Normalize z from 0.2 to 1.5 into a range [1, 100]
                    normalized_cost = (z - 0.2) / (1.5 - 0.2) * 100
                    occupancy_grid[grid_x, grid_y] = normalized_cost 
        return occupancy_grid
    
### A* start
    def a_star(self, start, goal):
        """
        A* pathfinding algorithm.
        """
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while not open_set.empty():
            _, current = open_set.get()

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    open_set.put((f_score[neighbor], neighbor))

        rospy.logwarn("A* failed to find a path")
        return []

    def heuristic(self, node, goal):
        """
        Heuristic function (Euclidean distance).
        """
        return np.linalg.norm(np.array(node) - np.array(goal))

    def get_neighbors(self, node):
        """
        Get neighboring cells in the grid.
        """
        neighbors = []
        x, y = node
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.occupancy_grid.shape[0] and 0 <= ny < self.occupancy_grid.shape[1]:
                if self.occupancy_grid[nx, ny] == 0:  # Free space
                    neighbors.append((nx, ny))
        return neighbors

    def reconstruct_path(self, came_from, current):
        """
        Reconstruct the path from A* results.
        """
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path

    def publish_waypoint(self, waypoint):
        """
        Publish a waypoint for the local planner.
        """
        wp = PoseStamped()
        wp.header.stamp = rospy.Time.now()
        wp.header.frame_id = "map"
        wp.pose.position.x, wp.pose.position.y = waypoint
        wp.pose.orientation.w = 1.0
        self.waypoint_pub.publish(wp)
### A* End

    def run(self):
        while not rospy.is_shutdown():
            if self.occupancy_grid is None:
                rospy.logwarn("Waiting for occupancy grid...")
                self.rate.sleep()
                continue

            start = (10, 10)  # Example starting point
            goal = (100, 100)  # Example goal point

            rospy.loginfo("Running A* algorithm...")
            path = self.a_star(start, goal)

            if path:
                rospy.loginfo(f"Path found: {path}")
                for waypoint in path:
                    self.publish_waypoint(waypoint)
                    rospy.sleep(1 / self.update_rate)  # Publish waypoints at desired frequency

            self.rate.sleep()


if __name__ == "__main__":
    try:
        planner = OctoMapAStar()
        planner.run()
    except rospy.ROSInterruptException:
        pass
    