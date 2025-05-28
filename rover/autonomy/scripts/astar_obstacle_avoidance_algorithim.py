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

If self.auto abort is still not working:
print it ou† to see.
"""

import rospy
import numpy as np
np.float = float
import ros_numpy
from nav_msgs.msg import Odometry, OccupancyGrid
from octomap_msgs.msg import Octomap
from geometry_msgs.msg import PoseStamped, Twist, Pose, Point, Quaternion
from sensor_msgs.msg import PointCloud2
#from octree import OctreeNode
#from Octomap import Octree
from queue import PriorityQueue
from std_msgs.msg import Header, Float32MultiArray
import math
from visualization_msgs.msg import Marker,MarkerArray
import tf.transformations as tf
from tf.transformations import quaternion_from_euler
import threading
from threading import Lock

from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, String, Bool
from nav_msgs.msg import Path
import os
import yaml
import time

file_path = os.path.join(os.path.dirname(__file__), "sm_config.yaml")

with open(file_path, "r") as f:
    sm_config = yaml.safe_load(f)


class AstarObstacleAvoidance():
    def __init__(self, lin_vel = 0.3, ang_vel= 0.3, goal=[(5,0)]):
        
        if sm_config.get("realsense_detection"):
            self.pointcloud_topic = rospy.get_param("~pointcloud_topic", sm_config.get("realsense_pointcloud"))
        else:
            self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "/zed_node/point_cloud/cloud_registered")
          
        # Parameters
        self.map_topic = rospy.get_param("~map_topic", "/octomap_binary")
        self.waypoint_topic = rospy.get_param("~waypoint_topic", "/waypoint")
        self.update_rate = rospy.get_param("~update_rate", 5.0)  # Frequency in Hz 
        self.vel_topic = rospy.get_param("~vel_topic", "/cmd_vel")  # Velocity command
        self.height_min = rospy.get_param("~height_min", 0.2)  # Min height for obstacles
        self.height_max = rospy.get_param("~height_max", 15)  # Max height for obstacles
        self.octomap_topic = rospy.get_param("~octomap_topic", "/octomap_binary")
        self.tree_resolution = rospy.get_param("~resolution", 0.1)  # OctoMap resolution (meters)
        self.pose_topic = rospy.get_param("~pose_topic", "/robot_pose")
        
        # Map and planning variables
        self.robot_footprint=[]
        self.boundary= ((-100000, -1000000, -100000), (100000, 100000, 100000)) 
        self.occupancy_grid = None
        self.grid_resolution = 0.1 # Resolution of 2D grid (meters per cell)
        #self.grid_origin=(0.0,0.0)
        self.goal = goal
        self.obstacle_threshold = 100
        self.grid_size=(10000,10000)
        self.grid_origin_starter = (
            -(self.grid_size[0]* self.grid_resolution)/2,  # -100.0 meters (for 0.1m resolution)
            -(self.grid_size[1]* self.grid_resolution)/2  # -100.0 meters
        )
        self.grid_origin=None
        # self.grid_origin = (0.0, 0.0)
        self.rate = rospy.Rate(self.update_rate)
        # self.tree = OctreeNode(self.boundary, self.tree_resolution)
        self.current_position_x=0
        self.current_position_y=0
        self.current_position_z=0
        self.current_orientation_x=0
        self.current_orientation_y=0
        self.current_orientation_z=0
        self.got_callback=False #make this false
        self.abort_check = False
        self.heading=0
        self.current_corner_array = [
            Point(x=1.5, y=1.5, z=0),
            Point(x=1.5, y=-1.5, z=0),
            Point(x=-1.5, y=-1.5, z=0), # with 0.5, it produces green blocks!
            Point(x=-1.5, y=1.5, z=0) ]
        self.z_min = -0.25
        self.z_max = 3
        self.yaw = 0
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel
        
        # Publishers and Subscribers
       # self.odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        
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
        self.abort_sub = rospy.Subscriber("auto_abort_check", Bool, self.abort_callback)
        self.pose_subscriber = rospy.Subscriber('/pose', PoseStamped, self.pose_callback)
        
    def abort_callback(self,msg):
        self.abort_check = msg.data

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
        (self.roll, self.pitch, self.yaw) = tf.euler_from_quaternion([
                self.current_orientation_x,
                self.current_orientation_y,
                self.current_orientation_z,
                self.current_orientation_w
            ])

        self.heading = self.to_euler_angles(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)[2]
        self.publish_bounding_box()
    
    def pose_callback(self, msg):
        self.got_callback=True
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
        gx = int(round((x - self.grid_origin_starter[0]) / self.grid_resolution))
        gy = int(round((y - self.grid_origin_starter[1]) / self.grid_resolution))
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = gx * self.grid_resolution + self.grid_origin_starter[0]
        y = gy * self.grid_resolution + self.grid_origin_starter[1]
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
      #  print("in recpnstruct path", current)
        path = [current]  # Start with the goal node
       # print("came_from", came_from)
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
        
        while not open_set.empty() and not self.abort_check:
            _, current = open_set.get()
          
            # Skip nodes already processed
            if current in closed:
                print("in astr, current is closed")
                continue
            closed.add(current)

            if current == goal:
                #print("in a*, open set is closed", current)
                return self.reconstruct_path(came_from, current)
          #  print("in get neighbors in astar", current)
            for neighbor in self.get_neighbors(current):
                
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
    
    def navigate(self):
        need_replan = True
        last_position = (0, 0)
        self.current_path = []
        last_plan_time = rospy.Time.now()
        replan_interval = rospy.Duration(5)
        path_available = False
        first_time = True
        goal = self.world_to_grid(self.goal[0][0], self.goal[0][1])
        rate = rospy.Rate(50)
        threshold = 0.5  # meters  for each waypoint
        angle_threshold = 0.5  # radians for each waypoint
        threshold_goal = 1.5 #for final waypoint
        kp = 0.5  # Angular proportional gain
        target_reached_flag=False #when true, stop!
       
       #this makes sure the origin is the right origin before stating to plan
        while not self.got_callback:
            if self.abort_check:
                print("self.abort is true!")
                break
            # rate.sleep()
        self.grid_origin=(self.current_position_x, self.current_position_y)
     
        while not rospy.is_shutdown() and not target_reached_flag and not self.abort_check:
            print("Self abort check is ", self.abort_check)
            msg = Twist()
            
            if self.abort_check:
                print("self.abort is true!")
                break
            
            if self.occupancy_grid is None: 
                if self.abort_check:
                    break
                print("self.occupancy_grid for straught line is None")
                rate.sleep()
                continue
           
            if self.abort_check or target_reached_flag:
                break
            
            current_x, current_y = self.world_to_grid(self.current_position_x,self.current_position_y)
            final_goal_target_distance= math.sqrt((goal[0] - current_x) ** 2 + (goal[1] - current_y) ** 2)
            
            if final_goal_target_distance < threshold_goal or target_reached_flag:
                print("final goal target distance", final_goal_target_distance)
                target_reached_flag=True
                msg.linear.x = 0
                msg.angular.z = 0
                self.drive_publisher.publish(msg)
                break
            
            start = self.world_to_grid(self.current_position_x, self.current_position_y)

            #replanning conditions
            
            need_replan = False
            if rospy.Time.now() - last_plan_time > replan_interval:
                need_replan = True

            if last_position:
                dx = start[0] - last_position[0]
                dy = start[1] - last_position[1]
                if abs(dx) > 6 or abs(dy) > 6:
                    need_replan = True

            if (need_replan or first_time) and not target_reached_flag and not path_available and not self.abort_check:
                print(f"attempting to replan: need replan is {need_replan}, path available is {path_available} and target_Reached_falg is {target_reached_flag}")
                need_replan = False
                first_time = False
                
                if self.abort_check:
                    break
                print("doing astar", start, goal)
                
                path = self.a_star(start, goal)
                if self.abort_check:
                    break
                if path:
                    print("path found",path)
                    self.current_path = path
                    last_position = start
                    path_available = True
                    self.publish_waypoints(path)
            
            # if len(self.current_path)==1:
            #     path_available=False
            #     print("len path is 1")
            #     break
   
            # Follow waypoints
            while path_available and self.current_path and not self.abort_check and not target_reached_flag:
                print("self.abort_check in in inner loop is", self.abort_check)
                current_x, current_y=self.world_to_grid(self.current_position_x,self.current_position_y)
                final_goal_target_distance= math.sqrt((goal[0] - current_x) ** 2 + (goal[1] - current_y) ** 2)
                
                msg = Twist()
                gx, gy = self.current_path[0]
                target_x, target_y = self.grid_to_world(gx, gy) #target =current waypoint only, go
                
                if final_goal_target_distance < threshold_goal or target_reached_flag:
                    target_reached_flag = True
                    msg.linear.x = 0
                    msg.angular.z = 0
                    self.drive_publisher.publish(msg)
                    print(f"Reached final target: ({target_x}, {target_y})")
                    break
                        
                dx = target_x - self.current_position_x
                dy = target_y - self.current_position_y
                target_distance = math.sqrt((dx) ** 2 + (dy) ** 2)
                target_heading = math.atan2(dy, dx)
                # print("trying to get to target:", target_x, target_y, self.current_position_x, self.current_position_y)
                
                angle_diff = target_heading - self.heading
                
                if angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                elif angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
                
                if self.abort_check:
                    break
                if target_distance < threshold and not target_reached_flag:
                    #print("target distance", target_distance)
                    self.current_path.pop(0)
                    msg.linear.x = 0
                    msg.angular.z = 0
                    self.drive_publisher.publish(msg)
                    if self.abort_check:
                        break
                    if target_reached_flag:
                        break
                    rospy.loginfo(f"Reached waypoint. Proceeding to next. There are {len(self.current_path)} waypoints left.")
                    
                    if self.abort_check:
                        break
                    
                    continue
                
                #added this. can try to just break after..
                if not self.current_path or target_reached_flag: #or len(self.current_path)==1:
                    print("All waypoints completed.")
                    target_reached_flag = True
                    break
                
                if abs(angle_diff) <= angle_threshold:
                   # print("trying to move forward", target_distance, self.current_position_x, target_x)
                    msg.linear.x = self.lin_vel
                    msg.angular.z = 0
                else:
                    #print("rotating", angle_diff)
                    msg.linear.x = 0
                    msg.angular.z = angle_diff * kp
                    if abs(msg.angular.z) < 0.3:
                        msg.angular.z = 0.3 if msg.angular.z > 0 else -0.3

                self.drive_publisher.publish(msg)
                rate.sleep()
                
                #break maybe uncomment this??
            rate.sleep()
                      
if __name__ == "__main__":
    rospy.init_node("octomap_a_star_planner", anonymous=True)
    try:
        planner = AstarObstacleAvoidance()
        planner.navigate()
    except rospy.ROSInterruptException:
        pass