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
- OctoMap is one way to turn that into a map (an octree of occupied vs. free space). - OctoMap needs 3D points + ray-casts to build its occupancy tree.
- Octree: What it is: A data structure used to partition 3D space hierarchically. Each node in an octree can have up to 8 children, subdividing space into smaller cubes (voxels).
- OctoMapWhat it is: A library built on octrees, designed for robotic mapping and navigation.

If self.auto abort is still not working:
print it out to see.

Notes from Jiaxu (20260425):
- There are so many if statements checking for abort in every loop; surely there's some way to clean this up? 
- Maybe add a check_abort helper method and just call it every time instead of if statements?
"""

import rclpy 
from rclpy.node import Node
import numpy as np
np.float = float
#import ros_numpy

# Message handling
from nav_msgs.msg import Odometry, OccupancyGrid
from octomap_msgs.msg import Octomap
from geometry_msgs.msg import PoseStamped, Twist, Pose, Point, Quaternion
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray, String, Bool
from nav_msgs.msg import Path
from rover.msg import MissionState
from std_msgs.msg import Header, Float32MultiArray
from sensor_msgs_py import point_cloud2

#from octree import OctreeNode
#from Octomap import Octree
from queue import PriorityQueue
import heapq
import math
# from builtin_interfaces.msg import Duration
from rclpy.duration import Duration
from visualization_msgs.msg import Marker,MarkerArray
# import tf_transformations as tf
# from transformations import quaternion_from_euler
from transforms3d.euler import quat2euler
import threading
from threading import Lock

from rclpy.executors import MultiThreadedExecutor

import os
from rclpy.clock import Clock
import yaml
import time


file_path = os.path.join(os.path.dirname(__file__), "sm_config.yaml")

with open(file_path, "r") as f:
    sm_config = yaml.safe_load(f)


class AstarObstacleAvoidance(Node):
    def __init__(self, lin_vel = 1.0, ang_vel= 0.3, goal=[(5,0)]): # default lin/ang velocities and goal coordinates. 
        super().__init__('octomap_a_star_planner')
        self.get_logger().info("Starting A* Obstacle Avoidance Node")
        
        # if sm_config.get("realsense_detection"):
        #     self.pointcloud_topic = rospy.get_param("~pointcloud_topic", sm_config.get("realsense_pointcloud"))
        # else:
        #     self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "/zed_node/point_cloud/cloud_registered")
        
        #  parameters with default values
        self.declare_parameter('realsense_detection', False)
        self.declare_parameter('realsense_pointcloud', '/camera/depth/points') #obstacle avoidance points from realsense
        self.declare_parameter('pointcloud_topic', '/zed/zed_node/point_cloud/cloud_registered') #obsatcle avoidance points from zed

        realsense_enabled = self.get_parameter('realsense_detection').get_parameter_value().bool_value
        realsense_topic = self.get_parameter('realsense_pointcloud').get_parameter_value().string_value
        fallback_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value

        if realsense_enabled: #check which camera we're using
            self.pointcloud_topic = realsense_topic
        else:
            self.pointcloud_topic = fallback_topic
        
        # Declare all parameters
        self.declare_parameter("map_topic", "/octomap_binary")
        self.declare_parameter("waypoint_topic", "/waypoint")
        self.declare_parameter("update_rate", 5.0)
        self.declare_parameter("vel_topic", "/cmd_vel")
        self.declare_parameter("height_min", 0.2)
        self.declare_parameter("height_max", 15.0)
        self.declare_parameter("octomap_topic", "/octomap_binary")
        self.declare_parameter("resolution", 0.1)
        # self.declare_parameter("pose_topic", "/pose")

        # Retrieve parameter values
        self.map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
        self.waypoint_topic = self.get_parameter("waypoint_topic").get_parameter_value().string_value
        self.update_rate = self.get_parameter("update_rate").get_parameter_value().double_value
        self.vel_topic = self.get_parameter("vel_topic").get_parameter_value().string_value
        self.height_min = self.get_parameter("height_min").get_parameter_value().double_value
        self.height_max = self.get_parameter("height_max").get_parameter_value().double_value
        self.octomap_topic = self.get_parameter("octomap_topic").get_parameter_value().string_value
        self.tree_resolution = self.get_parameter("resolution").get_parameter_value().double_value
        # self.pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        
        # Map and planning variables
        self.robot_footprint=[]
        self.boundary= ((-100000, -100000, -100000), (100000, 100000, 100000)) 
        self.grid_resolution = 0.2 # Resolution of 2D grid (meters per cell)
        #self.grid_origin=(0.0,0.0)
        self.goal = goal
        self.obstacle_threshold = 100
        
        # SMALLER GRID SIZE FOR TESTING
        self.grid_size = (1000, 1000) #self.grid_size=(10000,10000)
        w, h = self.grid_size
        
        self.occupancy_grid = np.zeros((h, w), dtype=np.int8)
        self.height_grid = np.zeros((h, w), dtype=np.float32)
        
        # self.grid_origin_starter = (
        #     -(self.grid_size[0]* self.grid_resolution)/2,  # -100.0 meters (for 0.2m resolution)
        #     -(self.grid_size[1]* self.grid_resolution)/2  # -100.0 meters
        # )

        # self.grid_origin = None
        # self.grid_origin = (0.0, 0.0)
        # self.rate = rospy.Rate(self.update_rate)
        self.rate = self.create_rate(self.update_rate)
        # self.tree = OctreeNode(self.boundary, self.tree_resolution)
        self.current_position_x=float(0)
        self.current_position_y=float(0)
        self.current_position_z=float(0)
        self.current_orientation_x=float(0)
        self.current_orientation_y=float(0)
        self.current_orientation_z=float(0)
        # Store heading at time of last A* plan so we can detect large rotations
        # and force a replan if the rover has turned more than 180 degrees.
        self.last_plan_heading = None
        # radians threshold for forcing replan if heading changes by this amount
        # default set to 125 degrees as requested
        self.replan_yaw_threshold = math.radians(125.0)
        self.got_callback=False #make this false
        self.abort_check = False
        self.heading=float(0)
        self.current_corner_array = [
            Point(x=1.8, y=1.8, z=0.0),
            Point(x=1.8, y=-1.8, z=0.0),
            Point(x=-1.8, y=-1.8, z=0.0), # with 0.5, it produces green blocks!
            Point(x=-1.8, y=1.8, z=0.0) ]
        
        
        # MODIFYING THIS FOR OPTIMIZING COSTMAP: initially -0.25 and 3. 
        self.z_min = -0.1
        self.z_max = 3.0
        
        self.yaw = 0
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel
        self.global_waypoints = []  # list of (x, y) world coords provided by global planner
        
        # Publishers
        self.bounding_box_pub = self.create_publisher(Marker, "/rover_bounding_box", 10)
        self.invalid_pose_pub = self.create_publisher(Marker, "/invalid_pose_markers", 10)
        self.waypoint_pub = self.create_publisher(PoseStamped, self.waypoint_topic, 10)
        self.astar_pub = self.create_publisher(Float32MultiArray, "/astar_waypoints", 10)
        self.octomap_pub = self.create_publisher(Octomap, self.octomap_topic, 10)
        
        # Publisher for the 2D Costmap
        self.costmap_pub = self.create_publisher(OccupancyGrid, "/costmap", 10)
        
        self.marker_array_pub = self.create_publisher(MarkerArray, "/dwa_trajectories", 1)
        self.astar_marker_pub = self.create_publisher(MarkerArray, "/astar_waypoints_markers", 1)
        self.footprint_pub = self.create_publisher(Marker, "/robot_footprint", 1)
        self.drive_publisher = self.create_publisher(Twist, "/drive", 10) # publishing final drive instructions
        self.pub = self.create_publisher(MissionState, 'mission_state', 10)
        self.mission_state_sub = self.create_subscription(MissionState,'mission_state',self.feedback_callback, 10)
        # self.test_state = "GLOBAL_PATH_READY"

        # Subscribers
        self.pointcloud_sub = self.create_subscription(PointCloud2, self.pointcloud_topic, self.pointcloud_callback, 10)
        self.abort_sub = self.create_subscription(Bool, "/auto_abort_check", self.abort_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, "/pose", self.pose_callback, 10)
        self._nav_lock = threading.Lock()
        self._nav_event = threading.Event()
        self.nav_thread = None
        self.gs_active = False
        self.is_navigating = False # Toggles print_info_callback to start printing only when we are actively navigating to a goal.
        
        # Timer to publish the costmap at 0.5 Hz (every 2.0 seconds)
        self.costmap_timer = self.create_timer(2.0, self.publish_costmap)
        self.info_timer = self.create_timer(10.0, self.print_info_callback)

                
    def abort_callback(self,msg):
        self.abort_check = msg.data
        
        # Jiaxu 20260425: added a thing that stops the rover immediately when aborting. 
        if self.abort_check:
            self.get_logger().warn("Abort triggered!")
            
            # Stop the rover instantly from the callback
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.drive_publisher.publish(stop_msg)
    
    def feedback_callback(self, msg):
        # print("in Obstacle Avoidance feedback callback, msg.state:", msg.state)
        # self.get_logger().info(f"in Obstacle Avoidance feedback callback, msg.state: {msg.state}")
        # self.get_logger().info(f"HELLOOOOOOOOOOO {msg.state} in feedback callback, starting A* navigation")
        if msg.state == "GLOBAL_PATH_READY" or msg.state == "GLOBAL_PATH_GS_READY":
            
            if msg.state == "GLOBAL_PATH_GS_READY":
                self.gs_active= True
               
            self.current_state = getattr(msg, "current_state", "Location Selection")
            target_x = msg.current_goal.pose.position.x
            target_y = msg.current_goal.pose.position.y
            self.global_waypoints = getattr(msg, "global_planner_waypoints", [])  # ✅ Safe and correct
            with self._nav_lock: #new 
                self.target = [(target_x, target_y)] #had this
            self._nav_event.set() #new
            if self.nav_thread is None or not self.nav_thread.is_alive():
                # self.get_logger().info(f"check 2: {msg.state}")
                
                # Start a thread and enter nav_loop
                self.nav_thread = threading.Thread(target=self._nav_loop, daemon=True)
                self.nav_thread.start()

    def _global_waypoints_as_tuples(self):
        if not self.global_waypoints:
            return []

        flat_data = np.array(self.global_waypoints, dtype=np.float64)
        if flat_data.size % 2 != 0:
            self.get_logger().warn("global_planner_waypoints had an odd number of values; ignoring route")
            return []

        reshaped_data = flat_data.reshape(-1, 2)
        return [tuple(point) for point in reshaped_data]
    
    def _nav_loop(self):
        while rclpy.ok():
            self._nav_event.wait()   
            if not rclpy.ok():
                break
            # copy target under lock to avoid races
            with self._nav_lock:
                targets = list(self.target) if self.target else []
                
            # Question: why targets plural if there is only one target (defined in line 233)? )
            for tx, ty in targets:
                self.goal = [(tx, ty)]
                self.navigate()
                if self.abort_check:
                    break
            self._nav_event.clear()

    # from sensor_msgs_py import point_cloud2
    # import numpy as np

    def pointcloud_callback(self, msg):
        # print("in feedback for pointcloud")
        """
        Take ZED PointCloud2 → update OcTree with both occupied + free voxels → publish OctoMap.
        """
        
        # Clear the local costmap so old obstacles don't "drag" with the rover
        self.occupancy_grid.fill(0)
        self.height_grid.fill(0)
        
        #  rospy.loginfo("Received point cloud, processing…")
        w, h = self.grid_size
        # xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

        xyz = point_cloud2.read_points_numpy(
            msg,
            field_names=["x", "y", "z"],
            skip_nans=True
        ).astype(np.float32, copy=False)

        if xyz.size == 0:
            return

        # # 3. Filter out +Inf and -Inf values common in ZED cameras
        mask = np.isfinite(xyz).all(axis=1)
        xyz = xyz[mask]

        if xyz.size == 0:
            return

        # 4. Filter by Z-height (ignoring the floor and ceiling)
        z = xyz[:, 2]
        z_mask = (z > self.z_min) & (z < self.z_max)
        xyz = xyz[z_mask]
        
        if xyz.size == 0:
            return
        
        # 5. Vectorized conversion to grid coordinates (Rolling Window Style)
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        
        # Rotate local points by rover heading and shift to the center of the array
        gx = np.round((xyz[:, 0] * cos_yaw - xyz[:, 1] * sin_yaw) / self.grid_resolution).astype(np.int32) + (w // 2)
        gy = np.round((xyz[:, 0] * sin_yaw + xyz[:, 1] * cos_yaw) / self.grid_resolution).astype(np.int32) + (h // 2)
        z = xyz[:, 2]

        # 6. Filter points that fall outside the grid boundaries
        valid = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
        gx = gx[valid]
        gy = gy[valid]
        z = z[valid]

        if gx.size == 0:
            return

        # 7. Update the persistent map
        self.occupancy_grid[gy, gx] = self.obstacle_threshold
        np.maximum.at(self.height_grid, (gy, gx), z)

    def get_grid_origin(self):
        """
        Bottom-left world coordinate of the rolling costmap.
        The rover is at the center cell.
        """
        return (
            self.current_position_x - (self.grid_size[0] * self.grid_resolution) / 2.0,
            self.current_position_y - (self.grid_size[1] * self.grid_resolution) / 2.0
        )

    def odom_callback(self, msg):
       # print("in callback")
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
        (self.roll, self.pitch, self.yaw) = quat2euler([
                self.current_orientation_x,
                self.current_orientation_y,
                self.current_orientation_z,
                self.current_orientation_w
            ])
        

        self.heading = self.to_euler_angles(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, #to roll, pitch, yaw
                                            msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)[2]
        self.publish_bounding_box()
    
    def pose_callback(self, msg):
        # self.get_logger().info(f" in pose callback")
        self.got_callback=True
        self.current_position_x = msg.pose.position.x
        self.current_position_y = msg.pose.position.y
        self.current_orientation_w=msg.pose.orientation.w
        self.current_orientation_z=msg.pose.orientation.z
        self.current_orientation_x=msg.pose.orientation.x
        self.current_orientation_y=msg.pose.orientation.y
        self.heading = self.to_euler_angles(msg.pose.orientation.w, msg.pose.orientation.x, 
                                            msg.pose.orientation.y, msg.pose.orientation.z)[2]
        
        (self.roll, self.pitch, self.yaw) = quat2euler([
                self.current_orientation_x,
                self.current_orientation_y,
                self.current_orientation_z,
                self.current_orientation_w
            ])
        
        self.publish_bounding_box()
    
    def print_info_callback(self):
        """
        Periodically prints the current position, the active goal, 
        and the Euclidean distance remaining to the goal.
        """
        # Ensure we have received a pose and have an active goal
        if not self.got_callback or not self.goal or not self.is_navigating:
            return

        # Extract current coordinates
        curr_x = self.current_position_x
        curr_y = self.current_position_y

        # Extract goal coordinates (self.goal is a list of tuples)
        goal_x, goal_y = self.goal[0]

        # Calculate Euclidean distance
        distance = math.hypot(goal_x - curr_x, goal_y - curr_y)

        # Print the status cleanly to the ROS logger
        self.get_logger().info(
            f"\n--- Navigation Status ---\n"
            f"Current Position: ({curr_x:.2f}, {curr_y:.2f})\n"
            f"Current Goal:     ({goal_x:.2f}, {goal_y:.2f})\n"
            f"Distance to Goal: {distance:.2f} meters\n"
            f"-------------------------"
        )
    
    def to_euler_angles(self, w, x, y, z):
        angles = [0.0, 0.0, 0.0]  # [roll, pitch, yaw]

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
    
    
    # NOT CURRENTLY USED ANYWHERE. 
    def publish_octomap(self):
        """
        Publish the generated OctoMap as a ROS message.
        is octomap =octree
        """
        # rospy.loginfo("Publishing OctoMap...")
        octomap_msg = self.tree.writeBinaryMsg()
        header = Header()
        header.stamp =  self.get_clock().now().to_msg() 
        header.frame_id = "map"
        octomap_msg.header = header
        self.octomap_pub.publish(octomap_msg)
        
        
    def publish_costmap(self):
        """
        Converts the internal numpy occupancy grid into a nav_msgs/OccupancyGrid
        and publishes it for visualization in Foxglove or RViz.
        """
        if self.occupancy_grid is None:
            return

        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "map"  # Match the frame your markers use

        # Set the grid metadata
        grid_msg.info.resolution = float(self.grid_resolution)
        grid_msg.info.width = int(self.grid_size[0])
        grid_msg.info.height = int(self.grid_size[1])

        # The origin sets the pose of cell (0,0) in the world frame
        origin_x, origin_y = self.get_grid_origin()

        grid_msg.info.origin.position.x = float(origin_x)
        grid_msg.info.origin.position.y = float(origin_y)
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0  # Identity quaternion (no rotation)

        # Flatten the 2D numpy array to a 1D Python list of int8. 
        # Since your occupied cells are 100 and free cells are 0, this maps perfectly to ROS standards.
        grid_msg.data = self.occupancy_grid.flatten().tolist()

        self.costmap_pub.publish(grid_msg)
    
    def publish_trajectories(self, trajectories):
        marker_array = MarkerArray()
        marker_id = 0

        for traj in trajectories:
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = self.get_clock().now().to_msg()
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
            # line_marker.lifetime = Duration(seconds=5)

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
        
        if not waypoints or len(waypoints) % 2 != 0:
            self.get_logger().warn("Invalid or empty waypoints list provided to RViz publisher.")
            return
        
        marker_array = MarkerArray()
        print("waypoints in publisher", waypoints)

        # Points marker for waypoints (larger size)
        points_marker = Marker()
        points_marker.header.frame_id = "map"
        points_marker.header.stamp =  self.get_clock().now().to_msg()
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
        # points_marker.lifetime = Duration(seconds=10).to_msg()

        # Line strip connecting waypoints
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = self.get_clock().now().to_msg()
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
        # line_marker.lifetime = Duration(seconds=10).to_msg()

        # Populate both markers with waypoints
        for index in range(0, len(waypoints), 2):
            
            p = Point()
            p.x = float(waypoints[index])
            p.y = float(waypoints[index+1])
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
        marker.header.stamp = self.get_clock().now().to_msg()
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
    
    def grid_to_world_with_origin(self, gx, gy, origin):
        origin_x, origin_y = origin

        x = gx * self.grid_resolution + origin_x
        y = gy * self.grid_resolution + origin_y

        return x, y
    # def world_to_grid(self, x, y):
    #     # Offset by half the grid size so the rover is always at index [500, 500]
    #     gx = int(round((x - self.current_position_x) / self.grid_resolution)) + (self.grid_size[0] // 2)
    #     gy = int(round((y - self.current_position_y) / self.grid_resolution)) + (self.grid_size[1] // 2)
    #     return gx, gy

    # def grid_to_world(self, gx, gy):
    #     # Convert back to world coordinates relative to the rover's current pose
    #     x = (gx - (self.grid_size[0] // 2)) * self.grid_resolution + self.current_position_x
    #     y = (gy - (self.grid_size[1] // 2)) * self.grid_resolution + self.current_position_y
    #     return x, y
    
    # def world_to_grid(self, x, y):
    #     origin_x, origin_y = self.grid_origin_starter
    #     gx = int(round((x - origin_x) / self.grid_resolution))
    #     gy = int(round((y - origin_y) / self.grid_resolution))
    #     return gx, gy

    # def grid_to_world(self, gx, gy):
    #     origin_x, origin_y = self.grid_origin_starter
    #     x = gx * self.grid_resolution + origin_x
    #     y = gy * self.grid_resolution + origin_y
    #     return x, y
   
    def world_to_grid(self, x, y):
        origin_x, origin_y = self.get_grid_origin()

        gx = int(round((x - origin_x) / self.grid_resolution))
        gy = int(round((y - origin_y) / self.grid_resolution))

        return gx, gy


    def grid_to_world(self, gx, gy):
        origin_x, origin_y = self.get_grid_origin()

        x = gx * self.grid_resolution + origin_x
        y = gy * self.grid_resolution + origin_y

        return x, y

### A* start 
    # Update height_cost:
    def height_cost(self, current, neighbor):
        current_gx, current_gy = current
        neighbor_gx, neighbor_gy = neighbor
        
        # Check obstacles first
        if self.occupancy_grid[current_gy, current_gx] >= self.obstacle_threshold: #if there is an obstacle at this point
            return float('inf')
        if self.occupancy_grid[neighbor_gy, neighbor_gx] >= self.obstacle_threshold: #if there is an obstacle at this point
            return float('inf')
        
        # Use actual height difference
        height_diff = abs(self.height_grid[current_gy, current_gx] - self.height_grid[neighbor_gy, neighbor_gx])
        return height_diff + 1  # +1 for base movement cost
        
    def heuristic(self, node, goal): #h fucntion -> euclidean distance, measures how far apart start and goal are, both in (x, y)
        return math.hypot(node[0] - goal[0], node[1] - goal[1])

    def reconstruct_path(self, came_from, current):
      #  print("in recpnstruct path", current)
        path = [current]  # Start with the goal node
        self.get_logger().info(f"IN RECONSTRUCT 1")
       # print("came_from", came_from)
        while current in came_from:
            current = came_from[current]
            path.append(current)  # Append predecessors
        self.get_logger().info(f"IN RECONSTRUCT 2")
        path.reverse()  # Reverse to get start→goal
        # print("path", path)
        self.get_logger().info(f"A* found a path to the goal! {len(path)}")
        return path
      
    def a_star(self, start, goal):
        # self.get_logger().info("OVER HEREEEEEEEE 1, Starting A* Pathfinding")
        
        # Pre-check: If the goal itself is physically invalid, A* will freeze trying to find it.
        goal_world_x, goal_world_y = self.grid_to_world(goal[0], goal[1])
        if not self.is_pose_valid((goal_world_x, goal_world_y, self.yaw)):
            self.get_logger().warn("A* Abort: Goal is inside an obstacle or too close to one!")
            return []
        
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0} #keeps track of distance/cost from start
        f_score = {start: self.heuristic(start, goal)} #keeps track of overall distance/cost from start to goal
        closed = set()

        
        while open_set and not self.abort_check: #while we have points to go to?
            _, current = heapq.heappop(open_set)
          
            # Skip nodes already processed
            if current in closed:
                # self.get_logger().info(" OVER in ASTAR, current is closed")
                continue
            closed.add(current)

            if current == goal:
                #print("in a*, open set is closed", current)
                # self.get_logger().info("OVER HEREEEEEEEE 2, Starting A* Pathfinding")
                return self.reconstruct_path(came_from, current) #if processed until goal, return the constructed path
            # else:
            #     self.get_logger().info(f"current and goal is not the same, current: {current}, goal: {goal}")
          #  print("in get neighbors in astar", current)
            neighbors = self.get_neighbors(current)
            for neighbor in neighbors: #get coord neighbours
                
                if neighbor in closed:
                    continue  # Skip closed nodes, has been processed

                height_cost = self.height_cost(current, neighbor)
            
                # Note: diagonal movements should technically cost ~1.414, not 1
                is_diagonal = (abs(current[0] - neighbor[0]) == 1 and abs(current[1] - neighbor[1]) == 1)
                step_cost = 1.414 if is_diagonal else 1.0
                
                tentative_g = g_score[current] + height_cost + step_cost

                if tentative_g < g_score.get(neighbor, float('inf')): #if its a shorter path overall to get to this point (may be due to objects), add it to path
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal) #overall path cost
                    heapq.heappush(open_set, (f_score[neighbor], neighbor)) #puts the neighbour in the queue to be inspected
            
        self.get_logger().warn("A* failed to find a path")
        return []
    
    # Check if the rover's footprint at a given pose would collide with any obstacles in the occupancy grid.
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
        w, h = self.grid_size
        
        for dx, dy in [(-1,0), (1,0), (0,1), (0,-1), (-1,-1), (-1,1), (1,-1), (1,1)]:
            ngx = gx + dx
            ngy = gy + dy
            
            if 0 <= ngx < w and 0 <= ngy < h: # Check for boundary values
                # Convert neighbor to world coordinates
                nx, ny = self.grid_to_world(ngx, ngy)
                if self.is_pose_valid((nx, ny, current_yaw)):
                    neighbors.append((ngx, ngy))
        return neighbors
    
    def publish_invalid_pose_marker(self, x, y, z=0.1):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "invalid_pose"
        marker.id = int(self.get_clock().now().to_sec() * 1000) % 1000000  # give it a semi-unique ID
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
        marker.lifetime = Duration(seconds=8)  # markers disappear after 8 seconds

        self.invalid_pose_pub.publish(marker)

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
    def publish_waypoints(self, waypoints):
        # Create the Float32MultiArray message
        msg = Float32MultiArray()
        # Flatten the waypoint list (e.g., [(x1, y1), (x2, y2)] -> [x1, y1, x2, y2])
        # flattened_waypoints = [coord for waypoint in waypoints for coord in waypoint]
        
        flattened_waypoints = []
        for gx, gy in waypoints:
            x, y = self.grid_to_world(gx, gy) #transform into world coords
            flattened_waypoints.extend([x, y])
            
        msg.data = flattened_waypoints  # Set the data field with the flattened list
        self.astar_pub.publish(msg)
        
        self.get_logger().info(f"Published {len(waypoints)} waypoints. Flat array: {flattened_waypoints}")
        
        self.publish_waypoints_rviz(flattened_waypoints)  # Publish the waypoints for visualization    

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    def downsample_path(self, path, step=5):
        """
        Keeps every `step` waypoint from the A* path.
        Always keeps the first and last waypoint.
        """
        if not path:
            return []

        if len(path) <= 2:
            return path

        simplified = path[::step]

        # Make sure final goal is always included
        if simplified[-1] != path[-1]:
            simplified.append(path[-1])

        return simplified

    def navigate_old(self):
        self.get_logger().info(f"HEYYYYYYY  in navigate loop")
        need_replan = True
        last_position = (0, 0)
        self.current_path = []
        last_plan_time = self.get_clock().now()
        replan_interval = Duration(seconds=5)
        path_available = False
        segment_reached = False
        first_time = True
        # goal will be computed after we set the dynamic grid origin
        world_goal = (self.goal[0][0], self.goal[0][1])
        rate = self.create_rate(50)
        threshold = 0.5  # meters  for each waypoint
        angle_threshold = 0.5  # radians for each waypoint
        threshold_goal = 1.5 #for final waypoint
        kp = 0.25  # Angular proportional gain (reduced for slower turning)
        target_reached_flag=False #when true, stop!
       
        while not self.got_callback: #hasn't gotten pose coord yet
            if self.abort_check:
                # print("self.abort is true!")
                break
            rate.sleep()
        
        # self.grid_origin = (self.current_position_x, self.current_position_y) #set the grid origin to init/curr position
        # Ensure conversions use the robot-centered origin before computing grid coords
        route_waypoints = self._global_waypoints_as_tuples()
        # Compute the grid goal now that origin is set
        goal = self.world_to_grid(self.goal[0][0], self.goal[0][1])
        world_goal = (self.goal[0][0], self.goal[0][1])
        # Debug: log start and goal in grid coordinates to detect mismatches
        try:
            start_grid = self.world_to_grid(self.current_position_x, self.current_position_y)
        except Exception:
            start_grid = (None, None)
       
        if not route_waypoints:
            route_waypoints = [world_goal]
        
        for waypoint_index, (goal_x, goal_y) in enumerate(route_waypoints):
            if self.abort_check:
                break

            segment_goal = self.world_to_grid(goal_x, goal_y)
            segment_world_goal = (goal_x, goal_y)
            # self.current_path = []
            
            self.current_path = [segment_world_goal]
            path_available = False
            first_time = True
            last_position = None
            last_plan_time = self.get_clock().now()
            need_replan = True

            # self.get_logger().info(
            #     f"Planning local A* segment {waypoint_index + 1}/{len(route_waypoints)} to {segment_world_goal}"
            # )

            while rclpy.ok() and not self.abort_check:
                
                msg = Twist()

                if self.occupancy_grid is None:
                    rate.sleep()
                    continue

                world_segment_goal_distance = math.sqrt(
                    (segment_world_goal[0] - self.current_position_x) ** 2 +
                    (segment_world_goal[1] - self.current_position_y) ** 2
                )
                if world_segment_goal_distance < threshold_goal:
                    msg.linear.x = float(0)
                    msg.angular.z = float(0)
                    self.drive_publisher.publish(msg)
                    self.current_path.clear()
                    path_available = False
                    segment_reached = True

                    self.get_logger().info(
                        f"Reached global waypoint {waypoint_index + 1}/{len(route_waypoints)}: {segment_world_goal}"
                    )
                    break

                start = self.world_to_grid(self.current_position_x, self.current_position_y)
                segment_goal = self.world_to_grid(goal_x, goal_y)

                need_replan = first_time
                if self.get_clock().now() - last_plan_time > replan_interval:
                    need_replan = True

                if last_position:
                    dx = start[0] - last_position[0]
                    dy = start[1] - last_position[1]
                    if abs(dx) > 6 or abs(dy) > 6:
                        need_replan = True

                # If the robot has rotated more than or equal to 180 degrees
                # since the last plan, force a replan. Use normalized angle
                # difference to compute shortest rotation. Treat abs(diff) >= pi
                # as the threshold for a >180° turn.
                if self.last_plan_heading is not None:
                    yaw_diff = abs(self.normalize_angle(self.heading - self.last_plan_heading))
                    if yaw_diff >= self.replan_yaw_threshold - 1e-6:
                        need_replan = True
                        deg = math.degrees(yaw_diff)
                        self.get_logger().info(
                            f"Heading changed by >={math.degrees(self.replan_yaw_threshold):.0f}° (diff={deg:.1f}°); forcing replan"
                        )

                if (need_replan or not path_available) and not self.abort_check:
                   
                    first_time = False
                    plan_origin = self.get_grid_origin()
                    path_grid = self.a_star(start, segment_goal)

                    # path = self.a_star(start, segment_goal)
                    # self.get_logger().info(f"OVER HEREEEEEEEE 6{len(path)}")

                    if not path_grid: # If path is empty, fall back to direct global waypoint following
                        self.get_logger().warn(
                            f"Local A* failed for waypoint {waypoint_index + 1}/{len(route_waypoints)} at {segment_world_goal}; falling back to direct global waypoint"
                        )
                        # Fallback: follow the global waypoint directly as a single target.
                        # self.current_path = [segment_goal]
                        # self.current_path = [
                        #     self.grid_to_world_with_origin(gx, gy, plan_origin)
                        #     for gx, gy in path_grid
                        # ]
                        self.current_path = [segment_world_goal]
                        last_position = start
                        last_plan_time = self.get_clock().now()
                        path_available = True
                        self.publish_waypoints(self.current_path)
                        continue

                    # self.get_logger().info(f"Local path found for segment {waypoint_index + 1}: {path}")
                    # self.current_path = path
                    last_position = start
                    last_plan_time = self.get_clock().now()
                    # record heading at the time of successful plan
                    self.last_plan_heading = self.heading
                    path_available = True
                    self.publish_waypoints(self.current_path)

                while path_available and self.current_path and not self.abort_check:
                    
                    msg = Twist()
                    current_x, current_y = self.world_to_grid(self.current_position_x, self.current_position_y)
                    world_segment_goal_distance = math.sqrt(
                        (segment_world_goal[0] - self.current_position_x) ** 2 +
                        (segment_world_goal[1] - self.current_position_y) ** 2
                    )
                    if world_segment_goal_distance < threshold_goal:
                        msg.linear.x = float(0)
                        msg.angular.z = float(0)
                        self.drive_publisher.publish(msg)
                        break

                    # gx, gy = self.current_path[0]
                    # target_x, target_y = self.grid_to_world(gx, gy)
                    target_x, target_y = self.current_path[0]

                    dx = target_x - self.current_position_x
                    dy = target_y - self.current_position_y
                    target_distance = math.sqrt((dx) ** 2 + (dy) ** 2)
                    target_heading = math.atan2(dy, dx)

                    angle_diff = target_heading - self.heading
                    if angle_diff > math.pi:
                        angle_diff -= 2 * math.pi
                    elif angle_diff < -math.pi:
                        angle_diff += 2 * math.pi

                    if target_distance < threshold:
                        self.current_path.pop(0)
                        msg.linear.x = float(0)
                        msg.angular.z = float(0)
                        self.drive_publisher.publish(msg)
                        self.get_logger().info(
                            f"Reached waypoint. Proceeding to next. There are {len(self.current_path)} waypoints left."
                        )
                        continue

                    if abs(angle_diff) <= angle_threshold:
                        msg.linear.x = self.lin_vel
                        msg.angular.z = float(0)
                    else:
                        msg.linear.x = float(0)
                        msg.angular.z = angle_diff * kp
                        if abs(msg.angular.z) < 0.3:
                            msg.angular.z = 0.3 if msg.angular.z > 0 else -0.3

                    self.drive_publisher.publish(msg)
                    rate.sleep()

                # if not self.current_path or self.abort_check:
                #     path_available = False
                #     break
                if segment_reached or not self.current_path or self.abort_check:
                    path_available = False
                    break

            if self.abort_check:
                break

        stop_msg = Twist()
        stop_msg.linear.x = float(0)
        stop_msg.angular.z = float(0)
        self.drive_publisher.publish(stop_msg)

        resp = MissionState()
        resp.current_state = getattr(self, "current_state", "")
        if self.gs_active:
            resp.state = "GS_WAYPOINT_REACHED" if not self.abort_check else "GS_FAILED"
            self.get_logger().info(f"ASTAR IS READY WITH POINTS in GS WAYPOINT REACHED PUBLISHED")
        else:
                        # self.get_logger().info(f"ASTAR IN NAVIGATE ELSEEE")
            resp.state = "SLA_AVOIDANCE_DONE" if not self.abort_check else "SLA_FAILED"
            self.pub.publish(resp)
        return

    def navigate_second_version(self):
        self.get_logger().info("HEYYYYYYY in navigate loop")

        self.current_path = []

        replan_interval = Duration(seconds=5)
        rate = self.create_rate(50)

        threshold = 0.5        # distance to each local A* waypoint
        threshold_goal = 1.5   # distance to final/global waypoint
        angle_threshold = 1  # radians, changed for now for testing. 
        kp = 0.25

        # Wait until we have pose data
        while not self.got_callback:
            if self.abort_check:
                break
            rate.sleep()

        if self.abort_check:
            return

        # Global planner waypoints are WORLD coordinates
        route_waypoints = self._global_waypoints_as_tuples()
        world_goal = (self.goal[0][0], self.goal[0][1])

        if not route_waypoints:
            route_waypoints = [world_goal]

        for waypoint_index, (goal_x, goal_y) in enumerate(route_waypoints):
            if self.abort_check:
                break

            segment_world_goal = (goal_x, goal_y)

            self.current_path = []
            path_available = False
            first_time = True
            last_position = None
            last_plan_time = self.get_clock().now()
            segment_reached = False

            self.get_logger().info(
                f"Starting A* segment {waypoint_index + 1}/{len(route_waypoints)} "
                f"to world goal {segment_world_goal}"
            )

            while rclpy.ok() and not self.abort_check:
                msg = Twist()

                if self.occupancy_grid is None:
                    rate.sleep()
                    continue

                # Check if final/global waypoint is reached
                world_segment_goal_distance = math.sqrt(
                    (segment_world_goal[0] - self.current_position_x) ** 2 +
                    (segment_world_goal[1] - self.current_position_y) ** 2
                )

                if world_segment_goal_distance < threshold_goal:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.drive_publisher.publish(msg)

                    self.current_path.clear()
                    path_available = False
                    segment_reached = True

                    self.get_logger().info(
                        f"Reached global waypoint {waypoint_index + 1}/{len(route_waypoints)}: "
                        f"{segment_world_goal}"
                    )
                    break

                # Convert current rover pose and segment goal into GRID coordinates
                start = self.world_to_grid(
                    self.current_position_x,
                    self.current_position_y
                )

                segment_goal = self.world_to_grid(
                    segment_world_goal[0],
                    segment_world_goal[1]
                )

                need_replan = first_time

                if self.get_clock().now() - last_plan_time > replan_interval:
                    need_replan = True

                if last_position is not None:
                    dx_grid = start[0] - last_position[0]
                    dy_grid = start[1] - last_position[1]

                    if abs(dx_grid) > 6 or abs(dy_grid) > 6:
                        need_replan = True

                if self.last_plan_heading is not None:
                    yaw_diff = abs(self.normalize_angle(self.heading - self.last_plan_heading))

                    if yaw_diff >= self.replan_yaw_threshold - 1e-6:
                        need_replan = True
                        self.get_logger().info(
                            f"Heading changed by >= {math.degrees(self.replan_yaw_threshold):.0f}° "
                            f"(diff={math.degrees(yaw_diff):.1f}°); forcing replan"
                        )

                if (need_replan or not path_available) and not self.abort_check:
                    first_time = False

                    # IMPORTANT:
                    # Save the rolling-grid origin at planning time.
                    # Then convert the path into fixed WORLD coordinates.
                    plan_origin = self.get_grid_origin()
                    path_grid = self.a_star(start, segment_goal)
                    path_grid = self.downsample_path(path_grid, step=10)
                    self.get_logger().info(f"A* returned {len(path_grid)} grid waypoints")

                    if not path_grid:
                        self.get_logger().warn(
                            f"Local A* failed for waypoint "
                            f"{waypoint_index + 1}/{len(route_waypoints)} at {segment_world_goal}; "
                            f"falling back to direct global waypoint"
                        )

                        # Fallback path is already WORLD coordinates
                        self.current_path = [segment_world_goal]

                    else:
                        # Convert grid path into fixed WORLD coordinates once.
                        # Do NOT keep them as grid cells, because the grid origin moves with the rover.
                        self.current_path = [
                            self.grid_to_world_with_origin(gx, gy, plan_origin)
                            for gx, gy in path_grid
                        ]

                    last_position = start
                    last_plan_time = self.get_clock().now()
                    self.last_plan_heading = self.heading
                    path_available = True

                    # Publish path in WORLD coordinates for visualization/debugging.
                    flat_path = []
                    for wx, wy in self.current_path:
                        flat_path.extend([float(wx), float(wy)])

                    path_msg = Float32MultiArray()
                    path_msg.data = flat_path
                    self.astar_pub.publish(path_msg)
                    self.publish_waypoints_rviz(flat_path)

                # Follow current WORLD-coordinate path
                while path_available and self.current_path and not self.abort_check:
                    msg = Twist()

                    world_segment_goal_distance = math.sqrt(
                        (segment_world_goal[0] - self.current_position_x) ** 2 +
                        (segment_world_goal[1] - self.current_position_y) ** 2
                    )

                    if world_segment_goal_distance < threshold_goal:
                        msg.linear.x = 0.0
                        msg.angular.z = 0.0
                        self.drive_publisher.publish(msg)

                        self.current_path.clear()
                        path_available = False
                        segment_reached = True

                        self.get_logger().info(
                            f"Reached global waypoint {waypoint_index + 1}/{len(route_waypoints)}: "
                            f"{segment_world_goal}"
                        )
                        break

                    # self.current_path now stores WORLD coordinates
                    target_x, target_y = self.current_path[0]

                    dx = target_x - self.current_position_x
                    dy = target_y - self.current_position_y

                    target_distance = math.sqrt(dx ** 2 + dy ** 2)
                    target_heading = math.atan2(dy, dx)

                    angle_diff = self.normalize_angle(target_heading - self.heading)

                    if target_distance < threshold:
                        self.current_path.pop(0)

                        msg.linear.x = 0.0
                        msg.angular.z = 0.0
                        self.drive_publisher.publish(msg)

                        self.get_logger().info(
                            f"Reached local A* waypoint. "
                            f"{len(self.current_path)} local waypoints left."
                        )
                        continue

                    if abs(angle_diff) <= angle_threshold:
                        msg.linear.x = float(self.lin_vel)
                        msg.angular.z = 0.0
                    else:
                        msg.linear.x = 0.0
                        msg.angular.z = float(angle_diff * kp)

                    self.drive_publisher.publish(msg)
                    rate.sleep()

                if segment_reached:
                    break

                if not self.current_path:
                    path_available = False
                    break

            if self.abort_check:
                break

        # Final stop after all route waypoints, or after abort
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.drive_publisher.publish(stop_msg)

        resp = MissionState()
        resp.current_state = getattr(self, "current_state", "")
        if self.gs_active:
            resp.state = "GS_WAYPOINT_REACHED" if not self.abort_check else "GS_FAILED"
            self.get_logger().info("ASTAR finished: publishing GS result")
            self.pub.publish(resp)
        else:
            resp.state = "SLA_AVOIDANCE_DONE" if not self.abort_check else "SLA_FAILED"
            self.get_logger().info("ASTAR finished: publishing SLA result")
            self.pub.publish(resp)

        return        
    
    def navigate(self):
        self.get_logger().info("HEYYYYYYY in navigate loop")

        self.current_path = []

        replan_interval = Duration(seconds=5)
        rate = self.create_rate(50)

        threshold = 0.5        # meters for each local A* waypoint
        threshold_goal = 1.5   # meters for final/global waypoint
        angle_threshold = 0.5  # radians
        kp = 0.25

        def stop_rover():
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.drive_publisher.publish(stop_msg)

        def publish_result():
            self.is_navigating = False # Turns off the print_message_callback when rover is done navigation. 
            resp = MissionState()
            resp.current_state = getattr(self, "current_state", "")

            if self.gs_active:
                resp.state = "GS_WAYPOINT_REACHED" if not self.abort_check else "GS_FAILED"
                self.get_logger().info(f"Publishing navigation result: {resp.state}")
            else:
                resp.state = "SLA_AVOIDANCE_DONE" if not self.abort_check else "SLA_FAILED"
                self.get_logger().info(f"Publishing navigation result: {resp.state}")

            self.pub.publish(resp)

        def distance_to_world_point(point):
            return math.sqrt(
                (point[0] - self.current_position_x) ** 2 +
                (point[1] - self.current_position_y) ** 2
            )

        def grid_to_world_with_origin(gx, gy, origin):
            origin_x, origin_y = origin
            x = gx * self.grid_resolution + origin_x
            y = gy * self.grid_resolution + origin_y
            return x, y

        def publish_world_path(world_path):
            flat_path = []

            for wx, wy in world_path:
                flat_path.extend([float(wx), float(wy)])

            path_msg = Float32MultiArray()
            path_msg.data = flat_path
            self.astar_pub.publish(path_msg)

            self.publish_waypoints_rviz(flat_path)

        # Wait until we have pose data
        while not self.got_callback:
            if self.abort_check:
                stop_rover()
                publish_result()
                return
            rate.sleep()

        if self.abort_check:
            stop_rover()
            publish_result()
            return

        # Final target from mission state
        world_goal = (self.goal[0][0], self.goal[0][1])

        # IMPORTANT:
        # If the goal is already reached, publish immediately and exit.
        # Do not continue into A*.
        if distance_to_world_point(world_goal) < threshold_goal:
            stop_rover()
            self.current_path.clear()

            self.get_logger().info(
                f"Already within threshold of final goal {world_goal}. "
                f"Distance={distance_to_world_point(world_goal):.2f} m"
            )

            publish_result()
            return

        # Global planner waypoints are world coordinates
        route_waypoints = self._global_waypoints_as_tuples()

        if not route_waypoints:
            route_waypoints = [world_goal]
        else:
            # Make sure the final target is included.
            # This avoids finishing only at an intermediate global waypoint.
            last_wp = route_waypoints[-1]
            last_wp_dist_to_goal = math.sqrt(
                (last_wp[0] - world_goal[0]) ** 2 +
                (last_wp[1] - world_goal[1]) ** 2
            )

            if last_wp_dist_to_goal > threshold_goal:
                route_waypoints.append(world_goal)

        for waypoint_index, segment_world_goal in enumerate(route_waypoints):
            if self.abort_check:
                break

            self.current_path = []
            path_available = False
            first_time = True
            last_position = None
            last_plan_time = self.get_clock().now()

            self.get_logger().info(
                f"Starting A* segment {waypoint_index + 1}/{len(route_waypoints)} "
                f"to {segment_world_goal}"
            )

            while rclpy.ok() and not self.abort_check:
                msg = Twist()

                # Check final mission goal first, not just the current segment.
                # This is what makes GS_WAYPOINT_REACHED publish immediately.
                if distance_to_world_point(world_goal) < threshold_goal:
                    stop_rover()
                    self.current_path.clear()

                    self.get_logger().info(
                        f"Reached final goal {world_goal}. "
                        f"Distance={distance_to_world_point(world_goal):.2f} m"
                    )

                    publish_result()
                    return

                # If this intermediate segment waypoint is reached,
                # move to the next segment waypoint.
                segment_distance = distance_to_world_point(segment_world_goal)

                if segment_distance < threshold_goal:
                    stop_rover()
                    self.current_path.clear()
                    path_available = False

                    self.get_logger().info(
                        f"Reached segment waypoint {waypoint_index + 1}/{len(route_waypoints)}: "
                        f"{segment_world_goal}. Moving to next segment."
                    )

                    break

                if self.occupancy_grid is None:
                    rate.sleep()
                    continue

                # Convert current rover pose and current segment goal into grid coords
                start = self.world_to_grid(
                    self.current_position_x,
                    self.current_position_y
                )

                segment_goal = self.world_to_grid(
                    segment_world_goal[0],
                    segment_world_goal[1]
                )

                need_replan = first_time

                if self.get_clock().now() - last_plan_time > replan_interval:
                    need_replan = True

                if last_position is not None:
                    dx_grid = start[0] - last_position[0]
                    dy_grid = start[1] - last_position[1]

                    if abs(dx_grid) > 6 or abs(dy_grid) > 6:
                        need_replan = True

                if self.last_plan_heading is not None:
                    yaw_diff = abs(self.normalize_angle(self.heading - self.last_plan_heading))

                    if yaw_diff >= self.replan_yaw_threshold - 1e-6:
                        need_replan = True
                        self.get_logger().info(
                            f"Heading changed by >= {math.degrees(self.replan_yaw_threshold):.0f}° "
                            f"(diff={math.degrees(yaw_diff):.1f}°); forcing replan"
                        )

                if (need_replan or not path_available) and not self.abort_check:
                    first_time = False

                    # Save rolling grid origin at planning time
                    plan_origin = self.get_grid_origin()

                    path_grid = self.a_star(start, segment_goal)

                    self.get_logger().info(
                        f"A* returned {len(path_grid)} grid waypoints"
                    )

                    if not path_grid:
                        self.get_logger().warn(
                            f"Local A* failed for segment waypoint "
                            f"{waypoint_index + 1}/{len(route_waypoints)} at {segment_world_goal}; "
                            f"falling back to direct segment waypoint"
                        )

                        # Fallback is already a world coordinate
                        self.current_path = [segment_world_goal]

                    else:
                        # Convert grid path into fixed world coordinates once.
                        # Do not keep grid cells because the rolling grid origin moves.
                        self.current_path = [
                            grid_to_world_with_origin(gx, gy, plan_origin)
                            for gx, gy in path_grid
                        ]

                    last_position = start
                    last_plan_time = self.get_clock().now()
                    self.last_plan_heading = self.heading
                    path_available = True

                    publish_world_path(self.current_path)

                # Follow current world-coordinate path
                while path_available and self.current_path and not self.abort_check:
                    msg = Twist()

                    # Check final mission goal first
                    if distance_to_world_point(world_goal) < threshold_goal:
                        stop_rover()
                        self.current_path.clear()

                        self.get_logger().info(
                            f"Reached final goal {world_goal}. "
                            f"Distance={distance_to_world_point(world_goal):.2f} m"
                        )

                        publish_result()
                        return

                    # Check current segment waypoint
                    if distance_to_world_point(segment_world_goal) < threshold_goal:
                        stop_rover()
                        self.current_path.clear()
                        path_available = False

                        self.get_logger().info(
                            f"Reached segment waypoint {waypoint_index + 1}/{len(route_waypoints)}: "
                            f"{segment_world_goal}. Moving to next segment."
                        )

                        break

                    # self.current_path stores world coordinates
                    target_x, target_y = self.current_path[0]

                    dx = target_x - self.current_position_x
                    dy = target_y - self.current_position_y

                    target_distance = math.sqrt(dx ** 2 + dy ** 2)
                    target_heading = math.atan2(dy, dx)

                    angle_diff = self.normalize_angle(target_heading - self.heading)

                    if target_distance < threshold:
                        self.current_path.pop(0)

                        # EDITED: REMOVED STOP FOR EVERY WAYPOINT
                        # msg.linear.x = 0.0
                        # msg.angular.z = 0.0
                        # self.drive_publisher.publish(msg)

                        self.get_logger().info(
                            f"Reached local A* waypoint. "
                            f"{len(self.current_path)} local waypoints left."
                        )

                        continue

                    if abs(angle_diff) <= angle_threshold:
                        msg.linear.x = float(self.lin_vel)
                        msg.angular.z = 0.0
                    else:
                        msg.linear.x = 0.0
                        msg.angular.z = float(angle_diff * kp)

                        if abs(msg.angular.z) < 0.3:
                            msg.angular.z = 0.3 if msg.angular.z > 0 else -0.3

                    self.drive_publisher.publish(msg)
                    rate.sleep()

                # If local path emptied but segment was not reached,
                # replan instead of exiting the whole navigation.
                if not self.current_path and not self.abort_check:
                    path_available = False
                    first_time = True

            # Continue to next route waypoint unless aborted.
            if self.abort_check:
                break

        # Final check after route loop
        stop_rover()

        if self.abort_check:
            publish_result()
            return

        if distance_to_world_point(world_goal) < threshold_goal:
            self.get_logger().info(
                f"Finished route and final goal is within threshold: {world_goal}"
            )
            publish_result()
            return

        # If we got here, the route ended but final goal was not reached.
        # For safety, publish failure-ish result depending on mode.
        self.get_logger().warn(
            f"Navigation loop ended, but final goal was not within threshold. "
            f"Final distance={distance_to_world_point(world_goal):.2f} m"
        )

        resp = MissionState()
        resp.current_state = getattr(self, "current_state", "")

        if self.gs_active:
            resp.state = "GS_FAILED"
        else:
            resp.state = "SLA_FAILED"

        self.pub.publish(resp)
        return       
def main(args=None):
    import rclpy
    rclpy.init()
    executor = MultiThreadedExecutor()
    AOA = AstarObstacleAvoidance()
    executor.add_node(AOA)
    try:
        executor.spin()
    finally:
        AOA.destroy_node()
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()