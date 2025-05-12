#!/usr/bin/env python3

"""
A* path planning with OctoMap integration for obstacle avoidance.

Want to get map of surroundngs using octomap within 5m -> to process it we will give each block a value representing its cost. 
The cost depends on height, density -> we go to lowest cost.
Cost depends on height. follow threshold.
1 for height
1 for density 

make a trajectory visualizer in a different file 

Feb 8 update: 
- communication is established between a star algo and a star trajectory
- next steps: 
    make sure we can see trajectory in RVIZ
    ensure A* is producing right code

    - for everyr way point-> check foot print
    calulcate g for being on that cell. caluclate total cost. 

    next steps: 
    - fix foorpring -> make it trun with rover -> *try multiplying it by 10 
        -> to fix it -> have 4 points for the rover and make lines instead of a box. 
        -> have the center be 0,0. then have the 4 cornesrs based on that center.
        -> make the neter, and 4 points actual data suing odometry. 
        -> 

    - make footprint local to rover
    - use foot print in A*!
    - try outputing the map ur mkaing
    Last edit before exam: 
    - had some filing issues. I made the corners global in an array: self.current_corners_array.
    next will use footprint in A*.
    For A*: 
    - I made all the 100000 to 1000.
    - I changed the height_cost function from height_cost_old to Height_cost_new. 
    this one makes it so that it assigns finite costs depending on the situation.
    - I also added a new function called: 'is pose_valid' and call it in new get neighbours function.
    - Added a new transform corner function taht is used in is pose valid. however, check whetehr you 
    should use local or global variables.

    I MADE BOTH LOCAL AND GLOBAL CORNER ARRAY

    query out of octomap depth. for loop in depth!
    look at check_collision in dwa_planner.cpp

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
import threading
from threading import Lock


class OctoMapAStar:
    def __init__(self):
        rospy.init_node("octomap_a_star_planner", anonymous=True)

        # Parameters
        self.map_topic = rospy.get_param("~map_topic", "/octomap_binary")
        self.waypoint_topic = rospy.get_param("~waypoint_topic", "/waypoint")
        self.update_rate = rospy.get_param("~update_rate", 5.0)  # Frequency in Hz 
        self.vel_topic = rospy.get_param("~vel_topic", "/cmd_vel")  # Velocity command
        self.height_min = rospy.get_param("~height_min", 0.2)  # Min height for obstacles
        self.height_max = rospy.get_param("~height_max", 15)  # Max height for obstacles
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "/zed/point_cloud/cloud_registered")
        self.octomap_topic = rospy.get_param("~octomap_topic", "/octomap_binary")
        self.tree_resolution = rospy.get_param("~resolution", 0.1)  # OctoMap resolution (meters)
        self.boundary= ((-100000, -1000000, -100000), (100000, 100000, 100000)) 
        self.pose_topic = rospy.get_param("~pose_topic", "/robot_pose")
        self.odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)

        self.robot_footprint=[]
        # Add a marker publisher
        self.bounding_box_pub = rospy.Publisher("/rover_bounding_box", Marker, queue_size=10)
        
        # Publishers and Subscribers
        self.map_sub = rospy.Subscriber(self.map_topic, Octomap, self.octomap_callback)
        #self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.odom_callback)
        self.waypoint_pub = rospy.Publisher(self.waypoint_topic, PoseStamped, queue_size=10)
        self.astar_pub = rospy.Publisher('/astar_waypoints', Float32MultiArray, queue_size=10)
        self.vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=10)
        self.pointcloud_sub = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.pointcloud_callback)
        self.octomap_pub = rospy.Publisher(self.octomap_topic, Octomap, queue_size=10)

        # Map and planning variables
        self.occupancy_grid = None
        self.grid_resolution = 1  # Resolution of 2D grid (meters per cell)
        self.grid_origin=(0.0,0.0)
        self.goal = (4,0)
        self.rate = rospy.Rate(self.update_rate)
        self.tree = OctreeNode(self.boundary, self.tree_resolution)
        self.current_position_x=0
        self.current_position_y=0 
        self.current_position_z=0
        self.current_orientation_x=0
        self.current_orientation_y=0
        self.current_orientation_z=0
        self.current_corner_array = [
    Point(x=0.4, y=0.4, z=0),
    Point(x=0.4, y=-0.4, z=0),
    Point(x=-0.4, y=-0.4, z=0),
    Point(x=-0.4, y=0.4, z=0)

] 
        
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
    
    def transform_robot_footprint(self, pose):
        """
        Transforms the robot's footprint from its local frame to the global frame.

        :param pose: A tuple (x, y, theta) representing the robot's current pose.
        :param robot_footprint: A list of (x, y) tuples representing the robot's footprint in local coordinates.
        :return: A list of transformed (x, y) tuples in global coordinates.
        """
        
        x_pose, y_pose, theta = pose
        
        for x_local, y_local in self.robot_foorprint:
            # Rotate the point
            x_global = x_local * math.cos(theta) - y_local * math.sin(theta)
            y_global = x_local * math.sin(theta) + y_local * math.cos(theta)

            # Translate the point
            x_global += x_pose
            y_global += y_pose

            self.robot_foorprint.append((x_global, y_global))
      
    def publish_octomap(self):
        """
        Publish the generated OctoMap as a ROS message.
        """
        # rospy.loginfo("Publishing OctoMap...")
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

    def decode_octomap(self, octomap_msg):
        """
        makes a list of occupied points in 3D space
        Decode an OctoMap binary message into a list of occupied points without using struct.
        """
       # rospy.loginfo("Decoding OctoMap...")
        rover_radius= 0.1
        resolution= 1.0
        inflation_cells = 0 #int ((rover_radius / (resolution)) )

        if octomap_msg.binary:
            data = octomap_msg.data
            resolution = octomap_msg.resolution

            # Initialize an empty list to store occupied points
            occupied_points = []

# free cell-free cell, #different obstacles # g1, g2, h. g1-> cost to get a point. g2-> cost at that point 
# because of obstacles. (g1 + g2 of every single cell before it and this one + h for this one + 
# everysingle cost in the footprint) #use foorprint..

            for offset, byte in enumerate(data):
                for bit in range(8):
                    if (byte >> bit) & 1:  # Check if the bit is set
                        # Calculate voxel index in 3D grid
                        voxel_index = offset * 8 + bit
                        x = (voxel_index % (256 * 256)) % 256
                        y = (voxel_index % (256 * 256)) // 256
                        z = voxel_index // (256 * 256)
            
                        x_real = x * resolution
                        y_real = y * resolution
                        z_real = z * resolution

                    # Add the main voxel to the list of occupied points
                        occupied_points.append((x_real, y_real, z_real))

                    # # Now apply inflation by adding the surrounding voxels to the list
                    #     for dx in range(-inflation_cells, inflation_cells + 1):
                    #         for dy in range(-inflation_cells, inflation_cells + 1):
                    #             for dz in range(-inflation_cells, inflation_cells + 1):
                    #                 # Compute the neighboring voxel positions considering inflation
                    #                 new_x = x + dx
                    #                 new_y = y + dy
                    #                 new_z = z + dz

                    #                 # Ensure the new voxel is within valid bounds (e.g., within the grid size)
                    #                 if 0 <= new_x < 256 and 0 <= new_y < 256 and 0 <= new_z < 256:
                    #                     # Convert the neighboring voxel to real-world coordinates and add it to occupied points
                    #                     occupied_points.append((new_x * resolution, new_y * resolution, new_z * resolution))

            rospy.loginfo(f"Decoded {len(occupied_points)} occupied points from OctoMap.")
            return occupied_points

    def publish_bounding_box(self):
        # note there is an older publish bounding box function that just makes a box around the rover..
        marker = Marker()
        marker.header.frame_id = "map"  # <<< ADD THIS LINE
       # rospy.loginfo("Publishing bounding box from corners")
       # marker.header.frame_id = "map"  # or "odom", depending on your TF setup
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
        gx = int((x - self.grid_origin[0]) / self.grid_resolution)
        gy = int((y - self.grid_origin[1]) / self.grid_resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = gx * self.grid_resolution + self.grid_origin[0]
        y = gy * self.grid_resolution + self.grid_origin[1]
        return x, y
    
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

        # Log the current position and orientation
        #rospy.loginfo("Current position: x=%f, y=%f, z=%f", 
          #          self.current_position_x, 
         #           self.current_position_y, 
        #            self.current_position_z)
        
       # rospy.loginfo("Current orientation (roll, pitch, yaw): roll=%f, pitch=%f, yaw=%f", 
           #         self.roll, self.pitch, self.yaw)
        
        # Publish the bounding box
        self.publish_bounding_box()

    def process_octomap(self, octomap_msg):
        """
        converts 3d map into 2d map with height filtering
        Process OctoMap into a 2D occupancy grid based on height values.
        """
        grid_size = (2000, 2000)  # Number of cells in x and y
              # Grid resolution in meters
        rover_radius = 0
        saftey_margin= 0
      ##inflation_cells = int ((rover_radius / (self.aftey_margin)
        
        occupancy_grid = np.zeros(grid_size, dtype=np.float32)
        occupied_points = self.decode_octomap(octomap_msg)

        for point in occupied_points:
            x, y, z = point
            # Convert to grid indices
            grid_x = int(round(x - self.grid_origin[0]) / self.grid_resolution)
            grid_y = int(round(y - self.grid_origin[1]) / self.grid_resolution)
    
            # Clamp indices to valid bounds
            if 0 <= grid_x < grid_size[0] and 0 <= grid_y < grid_size[1]:
                if z>0.3:
                    #Height threshold -< this should be 0.2<z<1.5!!!
                    #normalized_cost = int((z - 0.2) / (1.5 - 0.2) * 100)
                    occupancy_grid[grid_x, grid_y] = 1000 # 1000
                '''
                    for dx in range(-inflation_cells, inflation_cells):
                            for dy in range(-inflation_cells-1, inflation_cells+1):
                                # new_x = grid_x + dx 
                                # new_y = grid_y + dy   
                                new_x = min(max(grid_x + dx, 0), grid_size[0] - 1)
                                new_y = min(max(grid_y + dy, 0), grid_size[1] - 1)   
                                print("inflation_cells", inflation_cells) 
                                occupancy_grid[new_x, new_y] = 1000 
                '''
           # rospy.loginfo(f"Grid position: ({grid_x}, {grid_y}) -> Cost: {occupancy_grid[grid_x, grid_y]}")
        return occupancy_grid

### A* start 
    def height_cost(self, current, neighbor): #this is g function for height
        current_height = self.occupancy_grid[current[0], current[1]]
        neighbor_height = self.occupancy_grid[neighbor[0], neighbor[1]]
        # if current_height == -1 or neighbor_height == -1:  
        #     return 1000
        if current_height > 1000:
            return current_height
        elif neighbor_height >1000: 
            return neighbor_height
        
        return abs(current_height - neighbor_height) + 1
        
    def heuristic(self, node, goal): #h fucntion -> euclidean distance
        return np.linalg.norm(np.array(node) - np.array(goal))

    def reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.reverse()
        print("path", path)
        return path
    
    def a_star(self, start, goal):
        
        open_set = PriorityQueue()
        open_set.put((0, start))  #  start with f_score = 0
        came_from = {}
        g_score = {start: 0}  #  cost of getting to the start node is 0
        f_score = {start: self.heuristic(start, goal)}  # initial f_score based on heuristic

        while not open_set.empty():
            _, current = open_set.get()

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                height_cost = self.height_cost(current, neighbor)  #  height-based cost
              #  print("here is height_cost", height_cost)
                tentative_g_score = g_score[current] + height_cost  # Add the height cost to the g_score
                if height_cost < 1000: 
                  #  print("height cost is less than 1000")
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)  # f = g + h
                        open_set.put((f_score[neighbor], neighbor))
                        
        rospy.logwarn("A* failed to find a path")
        return []
    
    def is_pose_valid(self, pose):
        """Returns True if all corners of the footprint at this pose are in free space"""
        # print("in is pose valid", self.transform_corners(pose)
        for corner in self.transform_corners(pose):
            x, y = corner
           # print("occupancy grid shape:::",self.occupancy_grid.shape[0], self.occupancy_grid.shape[1])
           # print("corner x,y is ", x,y, self.grid_origin[1])
            grid_x =int (round(x- self.grid_origin[0])/self.grid_resolution) 
            grid_y =int(round (y - self.grid_origin[1])/ self.grid_resolution) 
         # print("for one point, this is self occupancy gird",self.occupancy_grid[grid_x, grid_y])
            # if not (0 <= grid_x < self.occupancy_grid.shape[0] and 0 <= grid_y < self.occupancy_grid.shape[1]):
            #     print("checkpoint 1 is true", grid_x, self.occupancy_grid.shape[0])
            #     return True  # out of bounds

            if self.occupancy_grid[grid_x, grid_y] >= 1000:
                print("checkpoint 2 true",x,y, grid_x, grid_y, pose, self.occupancy_grid[grid_x, grid_y])
                self.publish_invalid_pose_marker(x, y)  # Visualize in RViz
                return False  # This corner is in an obstacle
           # else: 
                #print("checkopoint 2 is false for grid_x, grid_y", grid_x, grid_y)
        return True
    
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
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(2.0)  # markers disappear after 2 seconds

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
    
    def get_neighbors(self, node):
        neighbors = []
        x, y = node
        current_yaw=self.yaw
        for dx, dy in [(-1, -1), (1, 1), (-1,1), (1,-1), (0, -1), (0, 1), (1, 0), (-1, 0)]:
            nx, ny = x + dx, y + dy
            # if 0 <= nx < self.occupancy_grid.shape[0] and 0 <= ny < self.occupancy_grid.shape[1]:
            # print("neighbours: ",nx, ny, x, y)
            if self.is_pose_valid((nx, ny,0)):
            #  print("pose is valid !!!!!!")
                neighbors.append((nx, ny))
            # else:
            #     print("Object detected")
        return neighbors
    
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

    def run(self):
        need_replan=True
        last_position=(0,0)
        last_plan_time = rospy.Time.now()
        replan_interval= rospy.Duration(3.0)
        while not rospy.is_shutdown():
            if self.occupancy_grid is None:
                print("self.occupancy_grid")
                self.rate.sleep()
                continue
            start =  self.world_to_grid(self.current_position_x, self.current_position_y) #(int(self.current_position_x), int(self.current_position_y))
            print("start", start)
            goal = self.goal  
            
            print("still in a*")
            if rospy.Time.now() - last_plan_time > replan_interval:

                print("need to replan because time passed")
                need_replan = True

            if last_position:
                dx = start[0] - last_position[0]
                dy = start[1] - last_position[1]
                if abs(dx) > 0.5 or abs(dy)>1:
                    print("need to replan because last_position moved")
                    need_replan = True

            if need_replan:
                need_replan=False
                print("attempting to replan")
                path = self.a_star(start, goal)
                if path:
                    # print("PATH Found", path)
                    # rospy.logwarn("A* found a path", path)
                    # rospy.loginfo(f"Path found: {path}")
                    current_pos = start
                    last_position = start
                    need_replan = False
                    path_available= path
                    
            if path_available and not need_replan:
                for waypoint in path:
                    self.publish_velocity(current_pos, waypoint)
                    current_pos = waypoint
                    rospy.sleep(1 / self.update_rate) 
                self.publish_waypoints(path)
            self.rate.sleep()     
            
    # def run_new(self):
    #     last_plan_time = rospy.Time.now()
    #     replan_interval = rospy.Duration(3.0)
    #     current_path = []
    #     current_wp_index = 0  # Track which waypoint we're pursuing
    #     path_lock = threading.Lock()  # Add this at class initialization

    #     while not rospy.is_shutdown():
    #         # Update grid origin with current position
    #         self.grid_origin = (self.current_position_x, self.current_position_y)
            
    #         # Get current grid positions
    #         start = self.world_to_grid(self.current_position_x, self.current_position_y)
    #         goal = self.goal

    #         # Replanning conditions
    #         need_replan = False
    #         if (rospy.Time.now() - last_plan_time > replan_interval or
    #             not current_path or
    #             current_wp_index >= len(current_path)):
    #             need_replan = True

    #         # Path following
    #         if current_path and current_wp_index < len(current_path):
    #             target_wp = current_path[current_wp_index]
    #             current_world = self.grid_to_world(*start)
    #             target_world = self.grid_to_world(*target_wp)
                
    #             # Calculate distance to waypoint
    #             dx = target_world[0] - current_world[0]
    #             dy = target_world[1] - current_world[1]
    #             distance = math.hypot(dx, dy)
                
    #             if distance > 0.1:
    #                 self.publish_velocity(current_world, target_world)
    #             else:
    #                 current_wp_index += 1  # Move to next waypoint

    #         # Replanning
    #         if need_replan:
    #             new_path = self.a_star(start, goal)
    #             if new_path:
    #                 with path_lock:
    #                     current_path = new_path
    #                     current_wp_index = 0  # Reset to first waypoint
    #                 last_plan_time = rospy.Time.now()
    #                 self.publish_waypoints(current_path)

    #         self.rate.sleep()

if __name__ == "__main__":
    try:
        planner = OctoMapAStar()
        planner.run()
    except rospy.ROSInterruptException:
        pass