#!/usr/bin/env python3

"""
A* path planning with OctoMap integration for obstacle avoidance.
This is straight line traversal!

Want to get map of surroundngs using octomap within 5m -> to process it we will give each block a value representing its cost. 
The cost depends on height, density -> we go to lowest cost.
Cost depends on height. follow threshold.

Potential note from Garvish: 
    query out of octomap depth. for loop in depth!
    look at check_collision in dwa_planner.cpp

Reim Notes Documentary: 
- PointClouds are the universal 3D data format; Zcam generates this.
- OctoMap is one way to turn that into a map (an octree of occupied vs. free space). - OctoMap needs 3D points + ray‐casts to build its occupancy tree.
- Octree: What it is: A data structure used to partition 3D space hierarchically. Each node in an octree can have up to 8 children, subdividing space into smaller cubes (voxels).
- OctoMapWhat it is: A library built on octrees, designed for robotic mapping and navigation.
"""

import rospy
import numpy as np
np.float = float
import time
import time
import ros_numpy
from nav_msgs.msg import Odometry, OccupancyGrid
from octomap_msgs.msg import Octomap
from geometry_msgs.msg import PoseStamped, Twist, Pose, Point, Quaternion
from sensor_msgs.msg import PointCloud2
#from octree import OctreeNode
#from Octomap import Octree
from queue import PriorityQueue
from std_msgs.msg import Header, Float32MultiArray, Float64MultiArray
from std_msgs.msg import Header, Float32MultiArray, Float64MultiArray
import math
from visualization_msgs.msg import Marker,MarkerArray
import tf.transformations as tf
from tf.transformations import quaternion_from_euler
import threading
from threading import Lock
import aruco_homing as aruco_homing
import ar_detection_node as ar_detect
import aruco_homing as aruco_homing
import ar_detection_node as ar_detect

from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, String, Bool
from std_msgs.msg import Float32MultiArray, String, Bool
from nav_msgs.msg import Path

import yaml
import os

file_path = os.path.join(os.path.dirname(__file__), "sm_config.yaml")

with open(file_path, "r") as f:
    sm_config = yaml.safe_load(f)
import yaml
import os

file_path = os.path.join(os.path.dirname(__file__), "sm_config.yaml")

with open(file_path, "r") as f:
    sm_config = yaml.safe_load(f)

class AstarObstacleAvoidance_GS_Traversal():
    def __init__(self, lin_vel = 0.3, ang_vel= 0.3, targets=[(1,0), (1,1)], state="AR3"):
class AstarObstacleAvoidance_GS_Traversal():
    def __init__(self, lin_vel = 0.3, ang_vel= 0.3, targets=[(1,0), (1,1)], state="AR3"):
          
        # Parameters
        self.map_topic = rospy.get_param("~map_topic", "/octomap_binary")
        self.waypoint_topic = rospy.get_param("~waypoint_topic", "/waypoint")
        self.update_rate = rospy.get_param("~update_rate", 5.0)  # Frequency in Hz 
        self.vel_topic = rospy.get_param("~vel_topic", "/cmd_vel")  # Velocity command
        self.height_min = rospy.get_param("~height_min", 0.2)  # Min height for obstacles
        self.height_max = rospy.get_param("~height_max", 15)  # Max height for obstacles
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "/zed_node/point_cloud/cloud_registered")
        self.octomap_topic = rospy.get_param("~octomap_topic", "/octomap_binary")
        self.tree_resolution = rospy.get_param("~resolution", 0.1)  # OctoMap resolution (meters)
        self.pose_topic = rospy.get_param("~pose_topic", "/robot_pose")
        
        # Map and planning variables
        self.robot_footprint=[]
        self.boundary= ((-100000, -1000000, -100000), (100000, 100000, 100000)) 
        self.occupancy_grid = None
        self.grid_resolution = 0.1 # Resolution of 2D grid (meters per cell)
        #self.grid_origin=(0.0,0.0)
        self.goal = (1,0)
        self.targets=targets
        self.state=state
        self.goal = (1,0)
        self.targets=targets
        self.state=state
        self.obstacle_threshold = 100
        self.grid_size=(10000,10000)
        self.grid_origin = (
            -(self.grid_size[0]* self.grid_resolution)/2,  # -100.0 meters (for 0.1m resolution)
            -(self.grid_size[1]* self.grid_resolution)/2  # -100.0 meters
        )
        # self.grid_origin = (0.0, 0.0)
        self.rate = rospy.Rate(self.update_rate)
        # self.tree = OctreeNode(self.boundary, self.tree_resolution)
        self.current_position_x=0
        self.current_position_y=0 
        self.current_position_z=0
        self.current_orientation_x=0
        self.current_orientation_y=0
        self.current_orientation_z=0
        self.abort_check = False
        self.heading=0
        self.current_corner_array = [
            Point(x=0.55, y=0.55, z=0),
            Point(x=0.55, y=-0.55, z=0),
            Point(x=-0.55, y=-0.55, z=0), # with 0.5, it produces green blocks!
            Point(x=-0.55, y=0.55, z=0) ]
        
        
        self.z_min = -0.25
        self.z_max = 3
        self.yaw = 0
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel

        self.found = False #Do we need?
        self.count = 0
        
        self.aruco_found = False
        self.mallet_found = False
        self.waterbottle_found = False

        # modified code: add dictionary to manage detection flags for multiple objects
        self.found_objects = {"AR1":False, 
                   "AR2":False,
                   "AR3":False,
                   "OBJ1":False,
                   "OBJ2":False}

        self.found = False #Do we need?
        self.count = 0
        
        self.aruco_found = False
        self.mallet_found = False
        self.waterbottle_found = False

        # modified code: add dictionary to manage detection flags for multiple objects
        self.found_objects = {"AR1":False, 
                   "AR2":False,
                   "AR3":False,
                   "OBJ1":False,
                   "OBJ2":False}
        
        # Publishers and Subscribers
     #   self.odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        self.bounding_box_pub = rospy.Publisher("/rover_bounding_box", Marker, queue_size=10)
        self.invaliid_pose_sub=rospy.Publisher('/invalid_pose_markers', Marker, queue_size=10)
        # self.map_sub = rospy.Subscriber(self.map_topic, Octomap, self.octomap_callback)
        # self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.odom_callback)
        self.waypoint_pub = rospy.Publisher(self.waypoint_topic, PoseStamped, queue_size=10)
        self.astar_pub = rospy.Publisher('/astar_waypoints', Float32MultiArray, queue_size=10)
        self.pointcloud_sub = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.pointcloud_callback)
        self.octomap_pub = rospy.Publisher(self.octomap_topic, Octomap, queue_size=10)
        self.marker_array_pub = rospy.Publisher('/dwa_trajectories', MarkerArray, queue_size=1)
        self.astar_marker_pub = rospy.Publisher('/astar_waypoints_markers', MarkerArray, queue_size=1)  # New publisher for A* markers
        self.footprint_pub = rospy.Publisher('/robot_footprint', Marker, queue_size=1)
        self.drive_publisher = rospy.Publisher('/drive', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/pose', PoseStamped, self.pose_callback)
        
        self.aruco_sub = rospy.Subscriber("aruco_found", Bool, callback=self.aruco_detection_callback)
        self.mallet_sub = rospy.Subscriber('mallet_detected', Bool, callback=self.mallet_detection_callback)
        self.waterbottle_sub = rospy.Subscriber('waterbottle_detected', Bool, callback=self.waterbottle_detection_callback)
        self.abort_sub = rospy.Subscriber("auto_abort_check", Bool, self.abort_callback)

    def abort_callback(self,msg):
        self.abort_check = msg.data
        
    def aruco_detection_callback(self, data):
        # print("sm grid search_ data.data", data.data)
        if data.data:
            if self.count <= 2:
                self.count +=1
            else:
                self.aruco_found = data.data
                self.found_objects[self.state] = data.data
                self.count += 1
    
    def mallet_detection_callback(self, data):
        if data.data:
            if self.count <= 2:
                self.count += 1
            else:
                self.mallet_found = data.data
                self.found_objects[self.state] = data.data
                self.count += 1

    def waterbottle_detection_callback(self, data):
        if data.data:
            if self.count <= 2:
                self.count += 1
            else:
                    
                self.waterbottle_found = data.data
                self.found_objects[self.state] = data.data
                self.count += 1  
    
    def pose_callback(self, msg):
        self.current_position_x = msg.pose.position.x
        self.current_position_y = msg.pose.position.y
        self.current_orientation_w=msg.pose.orientation.w
        self.current_orientation_z=msg.pose.orientation.z
        self.current_orientation_x=msg.pose.orientation.x
        self.current_orientation_y=msg.pose.orientation.y
        self.heading = self.to_euler_angles(msg.pose.orientation.w, msg.pose.orientation.x, 
                                            msg.pose.orientation.y, msg.pose.orientation.z)[2]
        
        (self.roll, self.pitch, self.yaw) = tf.euler_from_quaternion([
                self.current_orientation_x,
                self.current_orientation_y,
                self.current_orientation_z,
                self.current_orientation_w
            ])
        
    def pointcloud_callback(self, msg):
        """
        Take ZED PointCloud2 → update OcTree with both occupied + free voxels → publish OctoMap.
        """
        #  rospy.loginfo("Received point cloud, processing…")

        xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        mask = np.isfinite(xyz).all(axis=1)
        xyz = xyz[mask]

        w,h = self.grid_size
        new_height_grid = np.zeros((h, w), dtype=np.float32)  # heights
        grid= np.zeros((h, w), dtype=np.int8)   # shape = (rows, cols) #confirm shape!!!

        for x,y,z in xyz:
            if not (self.z_min < z < self.z_max):
               # print("not in threshold", z)
                continue

            gx,gy=self.world_to_grid(x,y)
            if 0 <= gx < w and 0 <= gy < h: #whithin the height threshhold so assign obstacle_threshold
                grid[gy, gx] = self.obstacle_threshold   # mark as occupied
                new_height_grid[gy, gx] = z  # Store actual height
                world_x, world_y = self.grid_to_world(gx, gy)       
                self.publish_invalid_pose_marker(world_x, world_y)  # Publish invalid pose marker
        
        self.height_grid=new_height_grid
        self.occupancy_grid=grid
    
    def odom_callback(self, msg):
            # Extract robot's position from the Odometry message
        self.current_position_x = msg.pose.pose.position.x
        self.current_position_y = msg.pose.pose.position.y
        self.current_position_z = msg.pose.pose.position.z

            # Extract robot's orientation (quaternion) from the Odometry message
        self.current_orientation_x = msg.pose.pose.orientation.x
        self.current_orientation_y = msg.pose.pose.orientation.y
        self.current_orientation_z = msg.pose.pose.orientation.z
        self.current_orientation_w = msg.pose.pose.orientation.w

            # Convert quaternion to Euler angles to get roll, pitch, and yaw (theta)
        (self.roll, self.pitch, self.yaw) = tf.euler_from_quaternion([
                self.current_orientation_x,
                self.current_orientation_y,
                self.current_orientation_z,
                self.current_orientation_w
            ])

        self.heading = self.to_euler_angles(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)[2]
        self.publish_bounding_box()
    
    def to_euler_angles(self, w, x, y, z):
        angles = [0, 0, 0]  # [roll, pitch, yaw]

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        angles[0] = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = math.sqrt(1 + 2 * (w * y - x * z))
        cosp = math.sqrt(1 - 2 * (w * y - x * z))
        angles[1] = 2 * math.atan2(sinp, cosp) - math.pi / 2

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        angles[2] = math.atan2(siny_cosp, cosy_cosp)
        return angles
    
    def publish_octomap(self):
        """
        Publish the generated OctoMap as a ROS message.
        is octomap =octree
        """
        # rospy.loginfo("Publishing OctoMap...")
        octomap_msg = self.tree.writeBinaryMsg()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        octomap_msg.header = header
        self.octomap_pub.publish(octomap_msg)
    
    def publish_trajectories(self, trajectories):
        marker_array = MarkerArray()
        marker_id = 0

        for traj in trajectories:
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = rospy.Time.now()
            line_marker.ns = "dwa_trajectories"
            line_marker.id = marker_id
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.02
            line_marker.color.r = 1.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0
            line_marker.pose.orientation.w = 1.0
            line_marker.lifetime = rospy.Duration(0.1)

            for point in traj:
                p = Point()
                p.x = point.x
                p.y = point.y
                p.z = 0.0
                line_marker.points.append(p)

            marker_array.markers.append(line_marker)
            marker_id += 1

        self.marker_array_pub.publish(marker_array)

    def publish_waypoints_rviz(self, waypoints):
        marker_array = MarkerArray()
        print("waypoints in publisher", waypoints)

        # Points marker for waypoints (larger size)
        points_marker = Marker()
        points_marker.header.frame_id = "map"
        points_marker.header.stamp = rospy.Time.now()
        points_marker.ns = "astar_points"
        points_marker.id = 0
        points_marker.type = Marker.POINTS
        points_marker.action = Marker.ADD
        points_marker.scale.x = 0.2  # Increased size
        points_marker.scale.y = 0.2
        points_marker.color.r = 1.0
        points_marker.color.g = 0.0
        points_marker.color.b = 0.0
        points_marker.color.a = 1.0
        points_marker.lifetime = rospy.Duration(0)

        # Line strip connecting waypoints
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "astar_line"
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.1  # Line width
        line_marker.color.r = 1.0
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        line_marker.pose.orientation.w = 1.0
        line_marker.lifetime = rospy.Duration(0)

        # Populate both markers with waypoints
        for index in range(0, len(waypoints), 2):
            
            p = Point()
            p.x = waypoints[index]
            p.y = waypoints[index+1]
            p.z = 0.0
            points_marker.points.append(p)
            line_marker.points.append(p)

        marker_array.markers.append(points_marker)
        marker_array.markers.append(line_marker)

        self.astar_marker_pub.publish(marker_array)
        
    def publish_bounding_box(self):
        # note there is an older publish bounding box function that just makes a box around the rover..
        marker = Marker()
        marker.header.frame_id = "map"  
        marker.header.stamp = rospy.Time.now()
        marker.ns = "rover_bounding_box"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.05  # Line width

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Transform corners to global frame using odometry pose
        pose = (self.current_position_x, self.current_position_y, self.yaw)
        global_corners = self.transform_corners(pose)

        # Convert back to geometry_msgs/Point
        marker.points = [Point(x=pt[0], y=pt[1], z=self.current_position_z) for pt in global_corners]
        # Close the loop by repeating the first point at the end
        marker.points.append(marker.points[0])
        self.bounding_box_pub.publish(marker)
    
    def world_to_grid(self, x, y):
        gx = int(round((x - self.grid_origin[0]) / self.grid_resolution))
        gy = int(round((y - self.grid_origin[1]) / self.grid_resolution))
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = gx * self.grid_resolution + self.grid_origin[0]
        y = gy * self.grid_resolution + self.grid_origin[1]
        return x, y
   
### A* start 
    # Update height_cost:
    def height_cost(self, current, neighbor):
        current_gx, current_gy = current
        neighbor_gx, neighbor_gy = neighbor
        
        # Check obstacles first
        if self.occupancy_grid[current_gy, current_gx] >= self.obstacle_threshold:
            return float('inf')
        if self.occupancy_grid[neighbor_gy, neighbor_gx] >= self.obstacle_threshold:
            return float('inf')
        
        # Use actual height difference
        height_diff = abs(self.height_grid[current_gy, current_gx] - self.height_grid[neighbor_gy, neighbor_gx])
        return height_diff + 1  # +1 for base movement cost
        
    def heuristic(self, node, goal): #h fucntion -> euclidean distance
        return np.linalg.norm(np.array(node) - np.array(goal))

    def reconstruct_path(self, came_from, current):
     #   print("in recpnstruct path", current)
        path = [current]  # Start with the goal node
      #  print("came_from", came_from)
        while current in came_from:
            current = came_from[current]
            path.append(current)  # Append predecessors
        path.reverse()  # Reverse to get start→goal
        print("path", path)
        return path
      
    def a_star(self, start, goal):
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        closed = set()

        while not open_set.empty():
            _, current = open_set.get()
          
            # Skip nodes already processed
            if current in closed:
                continue
            closed.add(current)

            if current == goal:
             #   print("in a*, open set is closed", current)
             #   print("in a*, open set is closed", current)
                return self.reconstruct_path(came_from, current)
          #  print("in get neighbors in astar", current)
            for neighbor in self.get_neighbors(current):
             #   print("going to get neighbors", neighbor)
             #   print("going to get neighbors", neighbor)
                if neighbor in closed:
                    continue  # Skip closed nodes

                height_cost = self.height_cost(current, neighbor)
                tentative_g = g_score[current] + height_cost

                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    open_set.put((f_score[neighbor], neighbor))

        rospy.logwarn("A* failed to find a path")
        return []
         
    def is_pose_valid(self, pose):
        for corner in self.transform_corners(pose):
            x, y = corner
            grid_x, grid_y = self.world_to_grid(x, y)
            
            if (0 <= grid_x < self.occupancy_grid.shape[1]) and (0 <= grid_y < self.occupancy_grid.shape[0]):
      
                if self.occupancy_grid[grid_y, grid_x] >= self.obstacle_threshold:
                  #  print("checkpoint 2 true", grid_x, grid_y, pose, self.occupancy_grid[grid_y, grid_x])
                    world_x, world_y = self.grid_to_world(grid_x, grid_y)
                   # self.publish_invalid_pose_marker(world_x, world_y)
                    return False
        return True
    
    def get_neighbors(self, node):
        neighbors = []
        gx, gy = node
        current_yaw = self.yaw
        
        # Convert node to world coordinates for footprint check
        wx, wy = self.grid_to_world(gx, gy)
        
        for dx, dy in [(-1,0), (1,0), (0,1), (0,-1), (-1,-1), (-1,1), (1,-1), (1,1)]:
            ngx = gx + dx
            ngy = gy + dy
            # Convert neighbor to world coordinates
            nx, ny = self.grid_to_world(ngx, ngy)
            if self.is_pose_valid((nx, ny, current_yaw)):
                neighbors.append((ngx, ngy))
        return neighbors
    
    def publish_invalid_pose_marker(self, x, y, z=0.1):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "invalid_pose"
        marker.id = int(rospy.Time.now().to_sec() * 1000) % 1000000  # give it a semi-unique ID
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(8.0)  # markers disappear after 2 seconds

        self.invaliid_pose_sub.publish(marker)

    def transform_corners(self, pose):
        """
        Transforms the current bounding box corners to the given (x, y, theta) pose.
        Returns a list of (x, y) tuples in global space.
        """
        x_pose, y_pose, theta = pose
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        transformed = []
        for pt in self.current_corner_array:
            local_x = pt.x
            local_y = pt.y

            x = local_x * cos_theta - local_y * sin_theta + x_pose
            y = local_x * sin_theta + local_y * cos_theta + y_pose

            transformed.append((x, y))
        return transformed
    
### A* END

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
   
    def publish_waypoints(self, waypoints):
        # Create the Float32MultiArray message
        msg = Float32MultiArray()

        # Flatten the waypoint list (e.g., [(x1, y1), (x2, y2)] -> [x1, y1, x2, y2])
        # flattened_waypoints = [coord for waypoint in waypoints for coord in waypoint]
        
        flattened_waypoints = []
        for gx, gy in waypoints:
            x, y = self.grid_to_world(gx, gy)
            flattened_waypoints.extend([x, y])
            
        msg.data = flattened_waypoints  # Set the data field with the flattened list
        self.astar_pub.publish(msg)
        # print("flattened waypoints", flattened_waypoints)
        self.publish_waypoints_rviz(flattened_waypoints)  # Publish the waypoints for visualization    

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def move_to_target(self, target_x, target_y, state):
    def move_to_target(self, target_x, target_y, state):
        need_replan = True
        last_position = (0, 0)
        self.current_path = []
        last_plan_time = rospy.Time.now()
        replan_interval = rospy.Duration(1.0)
        path_available = False
        first_time = True
        goal = self.world_to_grid(target_x, target_y)
        goal = self.world_to_grid(target_x, target_y)
        rate = rospy.Rate(50)
        threshold = 0.5  # meters
        angle_threshold = 0.5  # radians
        kp = 0.5  # Angular proportional gain
        
        obj = self.mallet_found or self.waterbottle_found
        
        mapping = {"AR1":self.aruco_found, 
                   "AR2":self.aruco_found,
                   "AR3":self.aruco_found,
                   "OBJ1":obj,
                   "OBJ2":obj}
        
        obj = self.mallet_found or self.waterbottle_found
        
        mapping = {"AR1":self.aruco_found, 
                   "AR2":self.aruco_found,
                   "AR3":self.aruco_found,
                   "OBJ1":obj,
                   "OBJ2":obj}

        while not rospy.is_shutdown():
            if self.occupancy_grid is None:
                print("self.occupancy_grid is None")
                rate.sleep()
                continue
            
            obj = self.mallet_found or self.waterbottle_found
        
            mapping = {"AR1":self.aruco_found, 
                    "AR2":self.aruco_found,
                    "AR3":self.aruco_found,
                    "OBJ1":obj,
                    "OBJ2":obj}
            
            if mapping[self.state] is False: #while not detected
                # normal operations
                if target_x is None or target_y is None or self.current_position_x is None or self.current_position_y is None:
                    continue   
                 
                start = self.world_to_grid(self.current_position_x, self.current_position_y)
                need_replan = False
                if rospy.Time.now() - last_plan_time > replan_interval:
                    need_replan = True
                continue
            
            obj = self.mallet_found or self.waterbottle_found
        
            mapping = {"AR1":self.aruco_found, 
                    "AR2":self.aruco_found,
                    "AR3":self.aruco_found,
                    "OBJ1":obj,
                    "OBJ2":obj}
            
            if mapping[self.state] is False: #while not detected
                # normal operations
                if target_x is None or target_y is None or self.current_position_x is None or self.current_position_y is None:
                    continue   
                 
                start = self.world_to_grid(self.current_position_x, self.current_position_y)
                need_replan = False
                if rospy.Time.now() - last_plan_time > replan_interval:
                    need_replan = True

                if last_position:
                    dx = start[0] - last_position[0]
                    dy = start[1] - last_position[1]
                    if abs(dx) > 0.5 or abs(dy) > 0.5:
                        need_replan = True
                if last_position:
                    dx = start[0] - last_position[0]
                    dy = start[1] - last_position[1]
                    if abs(dx) > 0.5 or abs(dy) > 0.5:
                        need_replan = True

                if need_replan or first_time:
                    print("attempting to replan", need_replan, path_available)
                    need_replan = False
                    first_time = False
                    path = self.a_star(start, goal)
                    if path:
                        print("path found",path)
                        self.current_path = path
                        last_position = start
                        path_available = True
                        self.publish_waypoints(path)
                        
                if need_replan or first_time:
                    print("attempting to replan", need_replan, path_available)
                    need_replan = False
                    first_time = False
                    path = self.a_star(start, goal)
                    if path:
                        print("path found",path)
                        self.current_path = path
                        last_position = start
                        path_available = True
                        self.publish_waypoints(path)
                        

                # Follow waypoints
                while path_available and self.current_path and not self.abort_check:

                    gx, gy = self.current_path[0]
                    target_x, target_y = self.grid_to_world(gx, gy)        
                    dx = target_x - self.current_position_x
                    dy = target_y - self.current_position_y
                    target_distance = math.sqrt((dx) ** 2 + (dy) ** 2)
                    target_heading = math.atan2(dy, dx)
                    
                    angle_diff = target_heading - self.heading
                    msg = Twist()
                # Follow waypoints
                while path_available and self.current_path and not self.abort_check:

                    gx, gy = self.current_path[0]
                    target_x, target_y = self.grid_to_world(gx, gy)        
                    dx = target_x - self.current_position_x
                    dy = target_y - self.current_position_y
                    target_distance = math.sqrt((dx) ** 2 + (dy) ** 2)
                    target_heading = math.atan2(dy, dx)
                    
                    angle_diff = target_heading - self.heading
                    msg = Twist()

                    if angle_diff > math.pi:
                        angle_diff -= 2 * math.pi
                    elif angle_diff < -math.pi:
                        angle_diff += 2 * math.pi
                        
                    if target_distance < threshold:
                        print("target distance", target_distance)
                        self.current_path.pop(0)
                        msg.linear.x = 0
                        msg.angular.z = 0
                        self.drive_publisher.publish(msg)
                        rospy.loginfo("Reached waypoint. Proceeding to next.")
                        continue
                    if angle_diff > math.pi:
                        angle_diff -= 2 * math.pi
                    elif angle_diff < -math.pi:
                        angle_diff += 2 * math.pi
                        
                    if target_distance < threshold:
                        print("target distance", target_distance)
                        self.current_path.pop(0)
                        msg.linear.x = 0
                        msg.angular.z = 0
                        self.drive_publisher.publish(msg)
                        rospy.loginfo("Reached waypoint. Proceeding to next.")
                        continue

                    if abs(angle_diff) <= angle_threshold:
                      #  print("trying to move forward", target_distance, self.current_position_x, target_x)
                        msg.linear.x = self.lin_vel
                        msg.angular.z = 0
                    else:
                       # print("rotating", angle_diff)
                        msg.linear.x = 0
                        msg.angular.z = angle_diff * kp
                        if abs(msg.angular.z) < 0.3:
                            msg.angular.z = 0.3 if msg.angular.z > 0 else -0.3

                    self.drive_publisher.publish(msg)
                    rate.sleep()
                    
            else: #HOMING!
                print("mapping state is true!")
                print("IN HOMING")
                # call homing
                # should publish that it is found
                # rospy.init_node('aruco_homing', anonymous=True) # change node name if needed
                pub = rospy.Publisher('drive', Twist, queue_size=10) # change topic name
                if self.state == "AR1" or self.state == "AR2" or self.state == "AR3":
                    aimer = aruco_homing.AimerROS(640, 360, 1000, 100, 100, sm_config.get("Ar_homing_lin_vel") , sm_config.get("Ar_homing_ang_vel")) # FOR ARUCO
                    rospy.Subscriber('aruco_node/bbox', Float64MultiArray, callback=aimer.rosUpdate) # change topic name
                    print (sm_config.get("Ar_homing_lin_vel"),sm_config.get("Ar_homing_ang_vel"))
                elif self.state == "OBJ1" or self.state == "OBJ2":
                    aimer = aruco_homing.AimerROS(640, 360, 1450, 100, 200, sm_config.get("Obj_homing_lin_vel"), sm_config.get("Obj_homing_ang_vel")) # FOR WATER BOTTLE
                    rospy.Subscriber('object/bbox', Float64MultiArray, callback=aimer.rosUpdate)
                    print (sm_config.get("Obj_homing_lin_vel"),sm_config.get("Obj_homing_ang_vel"))
                rate = rospy.Rate(10) #this code needs to be adjusted
                
                # Wait a bit for initial detection
                for i in range(50):
                    rate.sleep()
                
                # Add variables for tracking detection memory
                last_detection_time = time.time()
                detection_memory_duration = 2.0  # 2 seconds of memory
                detection_active = False
                
                while (not rospy.is_shutdown()) and (self.abort_check is False):
                    twist = Twist()
                    
                    # Check if we have valid values from the aimer
                    if aimer.linear_v is not None and aimer.angular_v is not None:
                        # We have a detection, update the timer
                        last_detection_time = time.time()
                        detection_active = True
                        
                        # Check if we've reached the target
                        if aimer.linear_v == 0 and aimer.angular_v == 0:
                            print ("at weird", aimer.linear_v, aimer.angular_v)
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                            pub.publish(twist)
                            return
                            
                        # Normal homing behavior
                        if aimer.angular_v == 1:
                            twist.angular.z = float(aimer.max_angular_v)
                          #  print ("first if",aimer.max_angular_v)
                            twist.linear.x = 0.0
                        elif aimer.angular_v == -1:
                            twist.angular.z = float(-aimer.max_angular_v)
                            twist.linear.x = 0.0
                        elif aimer.linear_v == 1:
                           # print ("second check",aimer.max_linear_v)
                            twist.linear.x = float(aimer.max_linear_v)
                            twist.angular.z = 0.0
                    else:
                        # No detection, check if we're within memory duration
                        if detection_active and time.time() - last_detection_time < detection_memory_duration:
                            print("Using last movement commands from memory")
                            # Continue with last valid movement
                            # (twist values are already set from previous iteration)
                        else:
                            # Memory expired, go back to grid search
                            print("Detection lost and memory expired, returning to grid search")
                            detection_active = False
                            break
                    
                    pub.publish(twist)
                    rate.sleep()

                break
            # print("publishing grid search velocity")
            self.drive_publisher.publish(msg)
            rate.sleep()
            
    def navigate(self): #navigate needs to take in a state value as well, default value is Location Selection
        for target_x, target_y in self.targets:
            if self.found_objects[self.state]: #should be one of aruco, mallet, waterbottle
                print(f"Object detected during navigation: {self.state}")
                return True
            
            print("Going to target", target_x, target_y)
            self.move_to_target(target_x, target_y, self.state) #changed from navigate_to_target
            if self.abort_check:
                break

            rospy.sleep(1)

        if self.found_objects[self.state]:
            return True
        return False #Signalling that grid search has been done


class GridSearch():
    '''
    All values here should be treated as doubles
    w, h - refers to grid dimensions, ie. if comp tells us that it is within radius of 20 then our grid would be square with 40 by 40
    tolerance   - this is the limitation of our own equipment, ie. from camera, aruco only detected left right and dist of 3 m
                - if this is unequal, take the smallest tolerance
    '''
    def __init__(self, w, h, tolerance, start_x, start_y): # AR1, OR AR2 (cartesian - fixed)
        self.x = w 
        self.y = h
        self.tol = tolerance 
        # print(self.tol)
        self.start_x = start_x
        self.start_y = start_y
        # rospy.Subscriber("/rtabmap/odom", Odometry, self.odom_callback) # change topic name

    '''
    Generates a list of targets for square straight line approach.
    '''
    def square_target(self):
        # dx, dy indicates direction (goes "up" first)
        dx, dy = 0, 1
        # while self.start_x == None:
        #     # print("Waiting for odom...")
        #     # rospy.loginfo("Waiting for odom...")
        #     continue
        targets = [(self.start_x,self.start_y)]
        step = 1

        while len(targets) < self.x:
            for i in range(2):  # 1, 1, 2, 2, 3, 3... etc
                for j in range(step):
                    if step*self.tol > self.x: 
                        print ("step is outside of accepted width: step*tol=", step*self.tol, "\tself.x=",self.x)
                        break
                    self.start_x, self.start_y = self.start_x + (self.tol*dx), self.start_y + (self.tol*dy)
                    targets.append((self.start_x, self.start_y))
                dx, dy = -dy, dx
            step += 1 
        print ("these are the final targets", targets) 
        return targets

    #Which function should be used?
    def alt_gs(self):
        dx, dy = 0, 1
        # while self.start_x == None:
        #     # print("Waiting for odom...")
        #     # rospy.loginfo("Waiting for odom...")
        #     continue
        targets = [(self.start_x,self.start_y)]
        step = 1
        # notice, up/down is odd amount, left/right is even amount        
        while len(targets) < self.x: # 1, 2, 3, 4, 5... sol 
            if step%2==1: # moving up/down
                for i in range (step):
                    # move up if %4==1, down if %4 ==3 from the pattern
                    self.start_y+=1 if step%4 ==1 else -1
                    targets.append((self.start_x,self.start_y))
                    if len(targets)==self.x:
                        return targets
            else: 
                for i in range (step):
                # move right if %4==2, left if %4 ==0 
                    self.start_x+=1 if step%4==2 else -1
                    targets.append((self.start_x,self.start_y))
                    if len(targets)==self.x:
                        return targets
            step += 1  # Increase step size after two edges
        return targets

                     
if __name__ == "__main__":
    rospy.init_node("octomap_a_star_planner", anonymous=True)
    try:
        planner = AstarObstacleAvoidance_GS_Traversal()
        planner = AstarObstacleAvoidance_GS_Traversal()
        planner.grid_search()
    except rospy.ROSInterruptException:
        pass