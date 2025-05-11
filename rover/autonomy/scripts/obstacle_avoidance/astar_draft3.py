#!/usr/bin/env python3
#new
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
from octomap import Octomap as OctomapAPI  # Requires python-octomap package

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
        try:
            # Convert ROS Octomap message to python-octomap object
            octomap = OctomapAPI()
            octomap.readBinary(msg.data)
            
            # Clear previous grid
            self.occupancy_grid = np.zeros(self.grid_size, dtype=np.float32)
            
            # Iterate through occupied cells
            iterator = octomap.begin_leafs_bbx(
                octomap.Point3d(-100, -100, 0.2),  # Min XYZ (0.2m height threshold)
                octomap.Point3d(100,100,2) )  # Max XYZ
            for node in iterator:
                if octomap.isNodeOccupied(node):
                    x = node.getCoordinate().x()
                    y = node.getCoordinate().y()
                    
                    # Convert to grid coordinates
                    grid_x = int((x - self.grid_origin[0]) / self.grid_resolution)
                    grid_y = int((y - self.grid_origin[1]) / self.grid_resolution)
                    
                    if 0 <= grid_x < self.grid_size[0] and 0 <= grid_y < self.grid_size[1]:
                        self.occupancy_grid[grid_x, grid_y] = 1000  # Obstacle cost
            
            rospy.loginfo("Updated occupancy grid with %d obstacles", np.sum(self.occupancy_grid > 0))
            
        except Exception as e:
            rospy.logerr(f"Octomap processing error: {str(e)}")

    # Rest of the class remains the same as previous version
    # (transform_corners, is_pose_valid, get_neighbors, heuristic, a_star, etc.)

if __name__ == "__main__":
    try:
        planner = OctoMapAStar()
        planner.run()
    except rospy.ROSInterruptException:
        pass