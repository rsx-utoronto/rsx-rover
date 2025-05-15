#!/usr/bin/env python3

"""
A* path planning with OctoMap integration for obstacle avoidance.

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
from std_msgs.msg import Float32MultiArray, String
from nav_msgs.msg import Path


class AstarObstacleAvoidance():
    def __init__(self, lin_vel = 0.5, ang_vel= 0.3, goal=(2,0)):
          
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
        self.goal = goal
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
            Point(x=0.3, y=0.3, z=0),
            Point(x=0.3, y=-0.3, z=0),
            Point(x=-0.3, y=-0.3, z=0), # with 0.5, it produces green blocks!
            Point(x=-0.3, y=0.3, z=0) ]
        self.z_min = -0.25
        self.z_max = 3
        self.yaw = 0
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel
        
        # Publishers and Subscribers
        self.odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        self.bounding_box_pub = rospy.Publisher("/rover_bounding_box", Marker, queue_size=10)
        self.invaliid_pose_sub=rospy.Publisher('/invalid_pose_markers', Marker, queue_size=10)
        #self.occ_pub=rospy.Publisher('/EDWARD', OccupancyGrid, queue_size=10)
        # self.map_sub = rospy.Subscriber(self.map_topic, Octomap, self.octomap_callback)
        # self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.odom_callback)
        self.waypoint_pub = rospy.Publisher(self.waypoint_topic, PoseStamped, queue_size=10)
        self.astar_pub = rospy.Publisher('/astar_waypoints', Float32MultiArray, queue_size=10)
        self.vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=10)
        self.pointcloud_sub = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.pointcloud_callback)
        self.octomap_pub = rospy.Publisher(self.octomap_topic, Octomap, queue_size=10)
        #self.frame_id = frame_id
        self.marker_array_pub = rospy.Publisher('/dwa_trajectories', MarkerArray, queue_size=1)
        self.astar_marker_pub = rospy.Publisher('/astar_waypoints_markers', MarkerArray, queue_size=1)  # New publisher for A* markers
        self.footprint_pub = rospy.Publisher('/robot_footprint', Marker, queue_size=1)
        self.drive_publisher = rospy.Publisher('/drive', Twist, queue_size=10)


    def pointcloud_callback(self, msg):
        """
        Take ZED PointCloud2 → update OcTree with both occupied + free voxels → publish OctoMap.
        """
       #  rospy.loginfo("Received point cloud, processing…")

        # 1) Get an (N×3) float32 array of xyz points, filter out NaNs
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
        print( "occupancy grid shape:::",self.occupancy_grid.shape[0], self.occupancy_grid.shape[1])
        #print("occuapncy grid shape:::",self.occupancy_grid.shape[0], self.occupancy_grid.shape[1])
    
    def odom_callback(self, msg):
        print("in odom callback")
        
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
        print("in recpnstruct path", current)
        path = [current]  # Start with the goal node
        print("came_from", came_from)
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
                print("in a*, open set is closed", current)
                return self.reconstruct_path(came_from, current)
          #  print("in get neighbors in astar", current)
            for neighbor in self.get_neighbors(current):
                print("going to get neighbors", neighbor)
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

    def publish_waypoints_yesterday(self, waypoints):
        # Create the Float32MultiArray message
        msg = Float32MultiArray()

        # Flatten the waypoint list (e.g., [(x1, y1), (x2, y2)] -> [x1, y1, x2, y2])
        #flattened_waypoints = [coord for waypoint in waypoints for coord in waypoint]
        
        flattened_waypoints = []
        for gx, gy in waypoints:
            x, y = self.grid_to_world(gx, gy)
            flattened_waypoints.extend([x, y])
            
        msg.data = flattened_waypoints  # Set the data field with the flattened list
        self.astar_pub.publish(msg)
        # print("flattened waypoints", flattened_waypoints)
        self.publish_waypoints_rviz(flattened_waypoints)  # Publish the waypoints for visualization

        rate = rospy.Rate(50)
        
        threshold = 0.5  # meters
        angle_threshold = 0.5  # radians
        kp = 0.5  # Angular proportional gain
        self.x=self.current_position_x
        self.y=self.current_position_y
        self.heading=self.yaw

        while not rospy.is_shutdown() and not self.abort_check:
            if not waypoints or self.x is None:
                rate.sleep()
                continue
            print("in move to target")
            current_target = waypoints[0]
            target_x, target_y = current_target

            # Calculate target direction and distance
            target_heading = math.atan2(target_y - self.y, target_x - self.x)
            target_distance = math.sqrt((target_x - self.x) ** 2 + (target_y - self.y) ** 2)
            
            angle_diff = target_heading - self.heading
            print("angle_diff", angle_diff,self.yaw)
            print("target_distance", target_distance)
            
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi  
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            # Create Twist message
            msg = Twist()

            if target_distance < threshold:
                waypoints.pop(0)
                msg.linear.x = 0
                msg.angular.z = 0
                self.drive_publisher.publish(msg)
                rospy.loginfo("Reached waypoint. Proceeding to next.")
                continue

            if abs(angle_diff) <= angle_threshold:
               # Move forward
                print("trying to move forward")
                print("target_distance", target_distance)
                msg.linear.x = self.lin_vel
                msg.angular.z = 0

            else:
                print("rotating", angle_diff)
                # Rotate in place
                msg.angular.z = max(min(kp * angle_diff, self.ang_vel), -self.ang_vel)

                # Minimum angular velocity to overcome friction
                if abs(msg.angular.z) < 0.3:
                    msg.angular.z = 0.3 if msg.angular.z > 0 else -0.3
                msg.linear.x = 0
                
            self.drive_publisher.publish(msg)
            rate.sleep()

        # Stop when done
        self.drive_publisher.publish(Twist())
    
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
        replan_interval = rospy.Duration(1.0)
        path_available = False
        first_time = True
        goal = self.world_to_grid(self.goal[0], self.goal[1])
        rate = rospy.Rate(50)
        threshold = 0.5  # meters
        angle_threshold = 0.5  # radians
        kp = 0.5  # Angular proportional gain

        while not rospy.is_shutdown():
            if self.occupancy_grid is None:
                print("self.occupancy_grid is None")
                rate.sleep()
                continue
            
            start = self.world_to_grid(self.current_position_x, self.current_position_y)
            need_replan = False
            print("in checkpoint 1")
            if rospy.Time.now() - last_plan_time > replan_interval:
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
                    print("path!",path)
                    self.current_path = path
                    last_position = start
                    path_available = True
                    self.publish_waypoints(path)
                    

            # Follow waypoints
            while path_available and self.current_path and not self.abort_check:
                
                gx, gy = self.current_path[0]
                
                target_x, target_y = self.grid_to_world(gx, gy)
                print("in path available", self.current_path, gx, gy, target_x, target_y)
                
                dx = target_x - self.current_position_x
                dy = target_y - self.current_position_y
                # target_distance = math.hypot(dx, dy)
                target_distance = math.sqrt((target_x - self.current_position_x) ** 2 + (target_y - self.current_position_y) ** 2)
                target_heading = math.atan2(dy, dx)
                
                #angle_diff = target_heading - self.yaw
                angle_diff = target_heading - self.heading
                # print("HERE IS YAW", self.yaw)
                print("here is heading", self.heading)
                msg = Twist()


                if angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                elif angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
                    
                if target_distance <= threshold:
                    print("target distance", target_distance)
                    self.current_path.pop(0)
                    msg.linear.x = 0
                    msg.angular.z = 0
                    self.drive_publisher.publish(msg)
                    rospy.loginfo("Reached waypoint. Proceeding to next.")
                    continue

                if abs(angle_diff) <= angle_threshold:
                    print("angle_diff" ,   angle_diff)
                    
                    print("trying to move forward", target_distance, self.current_position_x, target_x)
                    msg.linear.x = self.lin_vel
                    msg.angular.z = 0
                else:
                    print("rotating", angle_diff)
                    msg.linear.x = 0
                    msg.angular.z = angle_diff * kp
                    if abs(msg.angular.z) < 0.3:
                        msg.angular.z = 0.3 if msg.angular.z > 0 else -0.3

                    #  self.vel_pub.publish(msg)
                self.drive_publisher.publish(msg)
                rate.sleep()
        
       # self.drive_publisher.publish(Twist())  # Stop the rover when done
        

       
    def run(self):
        need_replan=True
        last_position=(0,0)
        self.current_path= []
        last_plan_time = rospy.Time.now()
        replan_interval= rospy.Duration(1.0)
        path_available=False
        first_time=True
        goal=self.world_to_grid(self.goal[0], self.goal[1])
        while not rospy.is_shutdown():
            if self.occupancy_grid is None:
                print("self.occupancy_grid is None")
                self.rate.sleep()
                continue
            print("running")
            start =  self.world_to_grid(self.current_position_x, self.current_position_y) #(int(self.current_position_x), int(self.current_position_y))
            need_replan=False
            if rospy.Time.now() - last_plan_time > replan_interval:
                print("need to replan because time passed")
                need_replan = True

            if last_position:
                dx = start[0] - last_position[0]
                dy = start[1] - last_position[1]
                if abs(dx) > 0.5 or abs(dy)>0.5:
                    print("need to replan because position changed")
                    need_replan = True

            if need_replan or first_time:
                need_replan=False
                first_time=False
                print("attempting to replan", need_replan, path_available)
                path = self.a_star(start, goal)
                if path:
                    current_pos = start
                    self.current_path=path
                    last_position = start
                    path_available = True
                    
                    for waypoint in path:
                        waypoint= self.grid_to_world(waypoint[0], waypoint[1])
                        self.publish_velocity(current_pos, waypoint)
                        current_pos = waypoint
                        rospy.sleep(1 / self.update_rate) 
                    self.publish_waypoints(path)
                   # self.publish_waypoints_rviz(path)
                    # print("path")
            self.rate.sleep()   
  
                
if __name__ == "__main__":
    rospy.init_node("octomap_a_star_planner", anonymous=True)
    try:
        planner = AstarObstacleAvoidance()
        planner.navigate()
    except rospy.ROSInterruptException:
        pass