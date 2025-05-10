#!/usr/bin/env python3

"""
A* path planning with OctoMap integration for obstacle avoidance.
"""

import rospy
import numpy as np
np.float = float

from nav_msgs.msg import Odometry
from octomap_msgs.msg import Octomap
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import PointCloud2
import ros_numpy
from octree import OctreeNode
from queue import PriorityQueue
from std_msgs.msg import Header
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker
import tf.transformations as tf

class OctoMapAStar:
    def __init__(self):
        rospy.init_node("octomap_a_star_planner", anonymous=True)

        # Parameters
        self.map_topic = rospy.get_param("~map_topic", "/octomap_binary")
        self.waypoint_topic = rospy.get_param("~waypoint_topic", "/waypoint")
        self.update_rate = rospy.get_param("~update_rate", 1.0)
        self.vel_topic = rospy.get_param("~vel_topic", "/cmd_vel")
        self.height_min = rospy.get_param("~height_min", 0.2)
        self.height_max = rospy.get_param("~height_max", 15)
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "/zed/point_cloud/cloud_registered")
        self.octomap_topic = rospy.get_param("~octomap_topic", "/octomap_binary")
        self.tree_resolution = rospy.get_param("~resolution", 0.1)
        self.boundary = ((-100000, -1000000, -100000), (100000, 100000, 100000)) 
        self.pose_topic = rospy.get_param("~pose_topic", "/robot_pose")
        self.odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)

        # Robot footprint parameters (half-length and half-width)
        self.footprint_length = 0.5
        self.footprint_width = 0.4
        
        # Publishers and Subscribers
        self.map_sub = rospy.Subscriber(self.map_topic, Octomap, self.octomap_callback)
        self.waypoint_pub = rospy.Publisher(self.waypoint_topic, PoseStamped, queue_size=10)
        self.astar_pub = rospy.Publisher('/astar_waypoints', Float32MultiArray, queue_size=10)
        self.vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=10)
        self.pointcloud_sub = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.pointcloud_callback)
        self.octomap_pub = rospy.Publisher(self.octomap_topic, Octomap, queue_size=10)
        self.bounding_box_pub = rospy.Publisher("/rover_bounding_box", Marker, queue_size=10)

        # Map and planning variables
        self.occupancy_grid = None
        self.grid_resolution = 0.1
        self.grid_origin = (0.0, 0.0)
        self.goal = (2.0, 0.0)
        self.rate = rospy.Rate(self.update_rate)
        self.tree = OctreeNode(self.boundary, self.tree_resolution)
        
        # Robot state variables
        self.current_position_x = 0
        self.current_position_y = 0 
        self.current_position_z = 0
        self.current_orientation_x = 0
        self.current_orientation_y = 0
        self.current_orientation_z = 0
        self.current_orientation_w = 1.0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    def pointcloud_callback(self, msg):
        rospy.loginfo("Received point cloud, processing...")
        point_cloud = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
        points = np.zeros((point_cloud.shape[0], 3), dtype=np.float32)

        points[:, 0] = point_cloud['x']
        points[:, 1] = point_cloud['y']
        points[:, 2] = point_cloud['z']

        valid_mask = np.isfinite(points).all(axis=1)
        points = points[valid_mask]

        for point in points:
            self.tree.updateNode(tuple(point), True)

        self.tree.updateInnerOccupancy()
        self.publish_octomap()
    
    def publish_octomap(self):
        rospy.loginfo("Publishing OctoMap...")
        octomap_msg = self.tree.writeBinaryMsg()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        octomap_msg.header = header
        self.octomap_pub.publish(octomap_msg)

    def octomap_callback(self, msg):
        octomap_data = self.process_octomap(msg)
        if octomap_data is not None:
            self.occupancy_grid = octomap_data 
            rospy.loginfo("2D occupancy grid generated from OctoMap.")

    def decode_octomap(self, octomap_msg):
        rospy.loginfo("Decoding OctoMap...")
        if octomap_msg.binary:
            data = octomap_msg.data
            resolution = octomap_msg.resolution
            occupied_points = []

            for offset, byte in enumerate(data):
                for bit in range(8):
                    if (byte >> bit) & 1:
                        voxel_index = offset * 8 + bit
                        x = (voxel_index % (256 * 256)) % 256
                        y = (voxel_index % (256 * 256)) // 256
                        z = voxel_index // (256 * 256)
            
                        x_real = x * resolution
                        y_real = y * resolution
                        z_real = z * resolution

                        occupied_points.append((x_real, y_real, z_real))

            rospy.loginfo(f"Decoded {len(occupied_points)} occupied points from OctoMap.")
            return occupied_points

    def publish_bounding_box(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "rover_bounding_box"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Get footprint corners in global frame
        corners = self.get_footprint_corners(
            (self.current_position_x, self.current_position_y, self.yaw),
            self.footprint_length,
            self.footprint_width
        )

        # Convert to Points and add to marker
        marker.points = [Point(x=pt[0], y=pt[1], z=self.current_position_z) for pt in corners]
        marker.points.append(marker.points[0])  # Close the loop

        self.bounding_box_pub.publish(marker)
    
    def get_footprint_corners(self, pose, length, width):
        """Returns the four corners of the robot footprint given a pose (x, y, theta)"""
        x, y, theta = pose
        half_l = length / 2
        half_w = width / 2
        
        # Define corners in local frame (relative to center)
        local_corners = [
            (half_l, half_w),   # Front right
            (half_l, -half_w),  # Front left
            (-half_l, -half_w), # Rear left
            (-half_l, half_w)    # Rear right
        ]
        
        # Transform to global frame
        global_corners = []
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        
        for (lx, ly) in local_corners:
            gx = lx * cos_theta - ly * sin_theta + x
            gy = lx * sin_theta + ly * cos_theta + y
            global_corners.append((gx, gy))
            
        return global_corners

    def odom_callback(self, msg):
        self.current_position_x = msg.pose.pose.position.x
        self.current_position_y = msg.pose.pose.position.y
        self.current_position_z = msg.pose.pose.position.z

        self.current_orientation_x = msg.pose.pose.orientation.x
        self.current_orientation_y = msg.pose.pose.orientation.y
        self.current_orientation_z = msg.pose.pose.orientation.z
        self.current_orientation_w = msg.pose.pose.orientation.w

        (self.roll, self.pitch, self.yaw) = tf.euler_from_quaternion([
            self.current_orientation_x,
            self.current_orientation_y,
            self.current_orientation_z,
            self.current_orientation_w
        ])
        
        self.publish_bounding_box()

    def process_octomap(self, octomap_msg):
        grid_size = (2000, 2000)
        resolution = 0.1
        
        occupancy_grid = np.zeros(grid_size, dtype=np.float32)
        occupied_points = self.decode_octomap(octomap_msg)

        for point in occupied_points:
            x, y, z = point
            grid_x = int((x - self.grid_origin[0]) / resolution)
            grid_y = int((y - self.grid_origin[1]) / resolution)
    
            if 0 <= grid_x < grid_size[0] and 0 <= grid_y < grid_size[1]:
                if z > 0.1:
                    occupancy_grid[grid_x, grid_y] = 1000

        return occupancy_grid

    ### A* Algorithm ###
    def height_cost(self, current, neighbor):
        current_height = self.occupancy_grid[current[0], current[1]]
        neighbor_height = self.occupancy_grid[neighbor[0], neighbor[1]]
        if current_height > 1000 or neighbor_height > 1000: 
            return float('inf')
        return abs(current_height - neighbor_height) + 1
        
    def heuristic(self, node, goal):
        return math.sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2)

    def reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path
    
    def a_star(self, start, goal):
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
                tentative_g_score = g_score[current] + height_cost
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    open_set.put((f_score[neighbor], neighbor))
                    
        rospy.logwarn("A* failed to find a path")
        return []
    
    def is_pose_valid(self, pose):
        """Check if the robot footprint at this pose is collision-free"""
        # Get footprint corners in global frame
        corners = self.get_footprint_corners(pose, self.footprint_length, self.footprint_width)
        
        for (x, y) in corners:
            # Convert to grid coordinates
            grid_x = int((x - self.grid_origin[0]) / self.grid_resolution)
            grid_y = int((y - self.grid_origin[1]) / self.grid_resolution)
            
            # Check bounds
            if (grid_x < 0 or grid_x >= self.occupancy_grid.shape[0] or 
                grid_y < 0 or grid_y >= self.occupancy_grid.shape[1]):
                return False
                
            # Check obstacle
            if self.occupancy_grid[grid_x, grid_y] >= 1000:
                return False
                
        return True

    def get_neighbors(self, node):
        neighbors = []
        x, y = node
        
        # 8-connected grid
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), 
                       (-1,-1), (-1,1), (1,-1), (1,1)]:
            nx, ny = x + dx, y + dy
            
            # Create pose (x, y, theta) - using current yaw for orientation
            pose = (nx * self.grid_resolution + self.grid_origin[0], 
                   ny * self.grid_resolution + self.grid_origin[1], 
                   self.yaw)
            
            if self.is_pose_valid(pose):
                neighbors.append((nx, ny))
                
        return neighbors

    ### Navigation ###
    def publish_velocity(self, current_pos, next_pos):
        velocity_msg = Twist()
        dx = next_pos[0] - current_pos[0]
        dy = next_pos[1] - current_pos[1]
        distance = math.sqrt(dx**2 + dy**2)

        if distance > 0.1:
            velocity_msg.linear.x = min(0.5, distance)
            angle = math.atan2(dy, dx)
            velocity_msg.angular.z = 2 * math.atan2(math.sin(angle), math.cos(angle))
        else:
            velocity_msg.linear.x = 0
            velocity_msg.angular.z = 0

        self.vel_pub.publish(velocity_msg)

    def publish_waypoints(self, waypoints):
        msg = Float32MultiArray()
        flattened_waypoints = [coord for waypoint in waypoints for coord in waypoint]
        msg.data = flattened_waypoints
        self.astar_pub.publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            if self.occupancy_grid is None:
                rospy.logwarn("Waiting for occupancy grid...")
                self.rate.sleep()
                continue
           
            start = (int(self.current_position_x / self.grid_resolution), 
                    int(self.current_position_y / self.grid_resolution))
            goal = (int(self.goal[0] / self.grid_resolution), 
                   int(self.goal[1] / self.grid_resolution))

            rospy.loginfo("Running A* algorithm...")
            path = self.a_star(start, goal)
            
            if path:
                rospy.loginfo(f"Path found: {path}")
                current_pos = (self.current_position_x, self.current_position_y)
            
                for waypoint in path:
                    next_pos = (waypoint[0] * self.grid_resolution + self.grid_origin[0],
                               waypoint[1] * self.grid_resolution + self.grid_origin[1])
                    self.publish_velocity(current_pos, next_pos)
                    current_pos = next_pos
                    rospy.sleep(1 / self.update_rate) 
                
                self.publish_waypoints(path)
                
            self.rate.sleep()           

if __name__ == "__main__":
    try:
        planner = OctoMapAStar()
        planner.run()
    except rospy.ROSInterruptException:
        pass