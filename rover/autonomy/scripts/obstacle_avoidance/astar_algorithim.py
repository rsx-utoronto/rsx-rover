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
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import PointCloud2
import ros_numpy
from octree import OctreeNode
from queue import PriorityQueue


class OctoMapAStar:
    def __init__(self):
        rospy.init_node("octomap_a_star_planner", anonymous=True)

        # Parameters
        self.map_topic = rospy.get_param("~map_topic", "/octomap_binary")
        self.waypoint_topic = rospy.get_param("~waypoint_topic", "/waypoint")
        self.update_rate = rospy.get_param("~update_rate", 1.0)  # Frequency in Hz 
        self.vel_topic = rospy.get_param("~vel_topic", "/cmd_vel")  # Velocity command
        self.height_min = rospy.get_param("~height_min", 0.2)  # Min height for obstacles
        self.height_max = rospy.get_param("~height_max", 15)  # Max height for obstacles
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "/zed/point_cloud/cloud_registered")
        self.octomap_topic = rospy.get_param("~octomap_topic", "/octomap_binary")
        self.tree_resolution = rospy.get_param("~resolution", 0.1)  # OctoMap resolution (meters)
        self.boundary= ((-1000, -1000, -10), (1000, 1000, 10)) 

        # Publishers and Subscribers
        self.map_sub = rospy.Subscriber(self.map_topic, Octomap, self.octomap_callback)
        self.waypoint_pub = rospy.Publisher(self.waypoint_topic, PoseStamped, queue_size=10)
        self.vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=10)
        self.pointcloud_sub = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.pointcloud_callback)
        self.octomap_pub = rospy.Publisher(self.octomap_topic, Octomap, queue_size=10)

        # Map and planning variables
        self.occupancy_grid = None
        self.grid_resolution = 0.1  # Resolution of 2D grid (meters per cell)
        self.grid_origin = None
        self.goal = None
        self.rate = rospy.Rate(self.update_rate)
        self.tree = OctreeNode(self.tree_resolution)
    
    def pointcloud_callback(self, msg):
            """
            Callback function to process the point cloud data and update the OctoMap.
            subscribes to the point cloud topic, processes the 3D points, and updates the OctoMap (a 3D occupancy grid).
            """
            rospy.loginfo("Received point cloud, processing...")

            # Convert ROS PointCloud2 message to numpy array
            point_cloud = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
            points = np.zeros((point_cloud.shape[0], 3), dtype=np.float32)

            # Extract x, y, z values from point cloud
            points[:, 0] = point_cloud['x']
            points[:, 1] = point_cloud['y']
            points[:, 2] = point_cloud['z']

            # Filter out invalid points (NaN or Inf)
            valid_mask = np.isfinite(points).all(axis=1)
            points = points[valid_mask]

            # Update the OctoMap tree with the valid points
            for point in points:
                self.tree.updateNode(tuple(point), True)

            # Update inner occupancy probabilities
            self.tree.updateInnerOccupancy()

            # Publish the OctoMap
            self.publish_octomap()

    def publish_octomap(self):
        """
        Publish the generated OctoMap as a ROS message.
        """
        rospy.loginfo("Publishing OctoMap...")
        octomap_msg = self.tree.writeBinaryMsg()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        octomap_msg.header = header
        self.octomap_pub.publish(octomap_msg)

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
        A* pathfinding algorithm with height cost.
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
                height_cost = self.height_cost(current, neighbor)
                tentative_g_score = g_score[current] + 1 + height_cost
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    open_set.put((f_score[neighbor], neighbor))

        rospy.logwarn("A* failed to find a path")
        return []

    def height_cost(self, current, neighbor):
        """
        Calculate the cost of height difference between two cells.
        """
        current_height = self.occupancy_grid[current[0], current[1]]
        neighbor_height = self.occupancy_grid[neighbor[0], neighbor[1]]
        return abs(current_height - neighbor_height)

    def heuristic(self, node, goal):
        """
        Heuristic function with height cost.
        """
        xy_distance = np.linalg.norm(np.array(node) - np.array(goal))
        height_difference = abs(self.occupancy_grid[node[0], node[1]] - self.occupancy_grid[goal[0], goal[1]])
        return xy_distance + height_difference

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

    def publish_velocity(self, current_pos, next_pos):
        """
        Publish velocity commands to move towards the next waypoint.
        speed will depend on how far away it is from somethin
        """
        velocity_msg = Twist()

        # Calculate the difference in position
        dx = next_pos[0] - current_pos[0]
        dy = next_pos[1] - current_pos[1]
        distance = math.sqrt(dx ** 2 + dy ** 2)

        # Set a simple proportional controller for speed
        if distance > 0.1:  # If the rover is not at the waypoint
            # Linear velocity based on distance
            velocity_msg.linear.x = min(0.5, distance)  # Limit max speed to 0.5 m/s

            # Angular velocity to align with the direction
            angle = math.atan2(dy, dx)
            velocity_msg.angular.z = 2 * math.atan2(math.sin(angle), math.cos(angle))  # Basic P-controller for angular velocity
        else:
            velocity_msg.linear.x = 0  # Stop when the waypoint is reached
            velocity_msg.angular.z = 0  # No rotation

        self.vel_pub.publish(velocity_msg)


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
            
            rospy.spin()
            if path:
                rospy.loginfo(f"Path found: {path}")
                current_pos = start
                for waypoint in path:
                    self.publish_velocity(current_pos, waypoint)
                    current_pos = waypoint
                    rospy.sleep(1 / self.update_rate)  # Publish velocity at desired frequency

            self.rate.sleep()
            

if __name__ == "__main__":
    try:
        planner = OctoMapAStar()
        planner.run()
    except rospy.ROSInterruptException:
        pass
    