#!/usr/bin/env python3

import rospy
import numpy as np
np.float = float
from nav_msgs.msg import Odometry
from octomap_msgs.msg import Octomap
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist, Point
from sensor_msgs.msg import PointCloud2
import ros_numpy
from queue import PriorityQueue
from std_msgs.msg import Header, Float32MultiArray
from visualization_msgs.msg import Marker
import math
import tf.transformations as tf

class OctoMapAStar:
    def __init__(self):
        rospy.init_node("octomap_a_star_planner")

        # Parameters
        self.grid_resolution = 1  # 1 meter per cell
        self.grid_size = (200, 200)  # 200x200 meter area
        self.grid_origin = (0.0, 0.0)
        self.footprint_half_length = 0.5  # 1m total length
        self.footprint_half_width = 0.4   # 0.8m total width

        # ROS Setup
        self.odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        self.map_sub = rospy.Subscriber("/octomap_binary", Octomap, self.octomap_callback)
        self.astar_pub = rospy.Publisher('/astar_waypoints', Float32MultiArray, queue_size=10)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.bounding_box_pub = rospy.Publisher("/rover_bounding_box", Marker, queue_size=10)

        # State variables
        self.occupancy_grid = np.zeros(self.grid_size, dtype=np.float32)
        self.current_position = (0.0, 0.0)
        self.yaw = 0.0

        # Footprint definition
        self.current_corner_array = [
            Point(self.footprint_half_length, self.footprint_half_width, 0),
            Point(self.footprint_half_length, -self.footprint_half_width, 0),
            Point(-self.footprint_half_length, -self.footprint_half_width, 0),
            Point(-self.footprint_half_length, self.footprint_half_width, 0)
        ]

    def odom_callback(self, msg):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        self.yaw = tf.euler_from_quaternion(quaternion)[2]
        self.publish_bounding_box()

    def octomap_callback(self, msg):
        temp_grid = np.zeros(self.grid_size, dtype=np.float32)
        
        try:
            point_cloud = ros_numpy.point_cloud2.pointcloud2_to_array(msg.data)
            points = np.zeros((point_cloud.shape[0], 3))
            points[:, 0] = point_cloud['x']
            points[:, 1] = point_cloud['y']
            points[:, 2] = point_cloud['z']
            
            for x, y, z in points:
                if z > 0.2:  # Obstacle height threshold
                    grid_x = int((x - self.grid_origin[0]) / self.grid_resolution)
                    grid_y = int((y - self.grid_origin[1]) / self.grid_resolution)
                    
                    if 0 <= grid_x < self.grid_size[0] and 0 <= grid_y < self.grid_size[1]:
                        temp_grid[grid_x, grid_y] = 1000  # Obstacle cost
                        
            self.occupancy_grid = temp_grid
            rospy.loginfo("Updated occupancy grid")
            
        except Exception as e:
            rospy.logerr(f"Octomap processing error: {str(e)}")

    def transform_corners(self, pose):
        x, y, theta = pose
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        return [
            (
                x + (pt.x * cos_t - pt.y * sin_t),
                y + (pt.x * sin_t + pt.y * cos_t)
            ) for pt in self.current_corner_array
        ]

    def is_pose_valid(self, grid_x, grid_y):
        real_x = self.grid_origin[0] + grid_x * self.grid_resolution
        real_y = self.grid_origin[1] + grid_y * self.grid_resolution
        corners = self.transform_corners((real_x, real_y, self.yaw))
        
        for x, y in corners:
            gx = int((x - self.grid_origin[0]) / self.grid_resolution)
            gy = int((y - self.grid_origin[1]) / self.grid_resolution)
            if not (0 <= gx < self.grid_size[0] and 0 <= gy < self.grid_size[1]):
                return False
            if self.occupancy_grid[gx, gy] >= 1000:
                return False
        return True

    def get_neighbors(self, node):
        neighbors = []
        grid_x, grid_y = node
        
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                
                nx = grid_x + dx
                ny = grid_y + dy
                
                if 0 <= nx < self.grid_size[0] and 0 <= ny < self.grid_size[1]:
                    if self.is_pose_valid(nx, ny):
                        neighbors.append((nx, ny))
        return neighbors

    def heuristic(self, a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def a_star(self, start, goal):
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while not open_set.empty():
            current = open_set.get()[1]
            
            if current == goal:
                return self.reconstruct_path(came_from, current)
                
            for neighbor in self.get_neighbors(current):
                tentative_g = g_score[current] + self.heuristic(current, neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    open_set.put((f_score[neighbor], neighbor))
                    
        return []

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def publish_bounding_box(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        corners = self.transform_corners((*self.current_position, self.yaw))
        for x, y in corners:
            p = Point()
            p.x = x
            p.y = y
            marker.points.append(p)
        marker.points.append(marker.points[0])  # Close the loop
        
        self.bounding_box_pub.publish(marker)

    def publish_waypoints(self, path):
        msg = Float32MultiArray()
        waypoints = []
        
        for grid_x, grid_y in path:
            real_x = self.grid_origin[0] + grid_x * self.grid_resolution
            real_y = self.grid_origin[1] + grid_y * self.grid_resolution
            waypoints.extend([real_x, real_y])
        
        msg.data = waypoints
        self.astar_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if np.sum(self.occupancy_grid) == 0:
                rate.sleep()
                continue
                
            start_x = int((self.current_position[0] - self.grid_origin[0]) / self.grid_resolution)
            start_y = int((self.current_position[1] - self.grid_origin[1]) / self.grid_resolution)
            start = (start_x, start_y)
            
            goal = (start_x + 5, start_y)  # 5 meters east
            
            path = self.a_star(start, goal)
            if path:
                self.publish_waypoints(path)
                
            rate.sleep()

if __name__ == "__main__":
    try:
        planner = OctoMapAStar()
        planner.run()
    except rospy.ROSInterruptException:
        pass