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


    new to do: 
    - print bounding box of rover in rviz
    - discuss octomap transformations
    - ensure rover avoids walla!

    - for eeveyr way point-> check foot print
    calulcate g for being on that cell. caluclate total cost. 

    next steps: 
    - fix foorpring -> make it trun with rover -> *try multiplying it by 10 
        -> to fix it -> have 4 points for the rover and make lines instead of a box. 
        -> have the center be 0,0. then have the 4 cornesrs based on that center.
        -> make the neter, and 4 points actual data suing odometry. 
        -> 

    - make footprint local to rover -> done 
    - use foot print in A*! 
    - try outputing the map ur mkaing
    - 

"""
import rospy
import numpy as np
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
        self.update_rate = rospy.get_param("~update_rate", 100.0)  # Frequency in Hz 
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
        self.grid_resolution = 0.1  # Resolution of 2D grid (meters per cell)
        self.grid_origin=(0.0,0.0)
        self.goal = (9.0,7.0)
        self.rate = rospy.Rate(self.update_rate)
        self.tree = OctreeNode(self.boundary, self.tree_resolution)
        self.current_position_x=0
        self.current_position_y=0 
        self.current_position_z=0
        self.current_orientation_x=0
        self.current_orientation_y=0
        self.current_orientation_z=0

        self.current_left_front_wheel_x = 0
        self.current_left_front_wheel_y = 0
        
        self.current_right_front_wheel_x = 0
        self.current_right_front_wheel_y = 0

        self.current_left_back_wheel_x = 0
        self.current_left_back_wheel_y = 0

        self.current_right_back_wheel_x = 0
        self.current_right_back_wheel_y=0
          
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

    def decode_octomap(self, octomap_msg):
        """
        makes a list of occupied points in 3D space
        Decode an OctoMap binary message into a list of occupied points without using struct.
        """
        rospy.loginfo("Decoding OctoMap...")
        rover_radius= 0.1
        resolution= 0.1
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
                        occupied_points.append((x * resolution, y * resolution, z * resolution))
            
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
        marker = Marker()
        marker.header.frame_id = "map"  # Adjust based on your TF setup
        marker.header.stamp = rospy.Time.now()
        marker.ns = "rover_bounding_box"
        marker.id = 0
        marker.type = Marker.LINE_STRIP  # Use LINE_STRIP to form a box
        marker.action = Marker.ADD

        # Set color and transparency
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully visible

        # Set scale (line thickness)
        marker.scale.x = 0.05  

        # Ensure rover starts at (0,0) and is aligned with axes
        if not hasattr(self, "initialized"):
            self.current_position_x = 0.0
            self.current_position_y = 0.0
            self.current_orientation_z = 0.0  # Ensures no initial rotation
            self.initialized = True  # Set a flag so it only resets once

        # Get the rover's yaw angle (assuming in radians)
        theta = self.current_orientation_z

        # Debugging: Print initial theta
        print(f"Initial theta: {theta}")

        # Normalize theta to the range [-π, π]
        theta = math.atan2(math.sin(theta), math.cos(theta))
        print(f"Normalized theta: {theta}")

        # Debugging: Print cos_theta and sin_theta
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        print(f"cos_theta: {cos_theta}, sin_theta: {sin_theta}")

        # Define the **fixed** corner positions relative to the rover's center
        half_width = 0.2 # Adjusted from 4 to 0.5
        original_corners = [
            (half_width,  half_width),   # Right Front
            (-half_width, half_width),   # Right Back
            (-half_width, -half_width),  # Left Back
            (half_width, -half_width),   # Left Front
            (half_width,  half_width)    # Closing the box
        ]

        # Apply rotation and translation
        transformed_corners = []
        for cx, cy in original_corners:
            x = cx * cos_theta - cy * sin_theta  # Rotate
            y = cx * sin_theta + cy * cos_theta  # Rotate

            x += self.current_position_x  # Translate
            y += self.current_position_y  # Translate

            transformed_corners.append(Point(x, y, 0))
            print(f"Transformed corner: ({x}, {y})")  # Debugging

        marker.points = transformed_corners  # Assign transformed points to marker

        # Publish the marker
        self.bounding_box_pub.publish(marker)


    
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

        # self.current_left_front_wheel_x=self.current_position_x + 0.5
        # self.current_left_front_wheel_y=self.current_position_y - 0.5
        
        # self.current_right_front_wheel_x = self.current_position_x + 0.5
        # self.current_right_front_wheel_y = self.current_position_y + 0.5

        # self.current_left_back_wheel_x =  self.current_position_x - 0.5
        # self.current_left_back_wheel_y = self.current_position_y - 0.5

        # self.current_right_back_wheel_x = self.current_position_x - 0.5
        # self.current_right_back_wheel_y= self.current_position_y + 0.5

        # Convert quaternion to Euler angles to get roll, pitch, and yaw (theta)
        (self.roll, self.pitch, self.yaw) = tf.euler_from_quaternion([
            self.current_orientation_x,
            self.current_orientation_y,
            self.current_orientation_z,
            self.current_orientation_w
        ])

        # Log the current position and orientation
        rospy.loginfo("Current position: x=%f, y=%f, z=%f", 
                    self.current_position_x, 
                    self.current_position_y, 
                    self.current_position_z)
        
        rospy.loginfo("Current orientation (roll, pitch, yaw): roll=%f, pitch=%f, yaw=%f", 
                    self.roll, self.pitch, self.yaw)
        
        # Publish the bounding box
        self.publish_bounding_box()

    def process_octomap(self, octomap_msg):
        """
        converts 3d map into 2d map with height filtering
        Process OctoMap into a 2D occupancy grid based on height values.
        """
        grid_size = (200, 200)  # Number of cells in x and y
        resolution = 0.1        # Grid resolution in meters
        rover_radius = 0
        saftey_margin= 0
        inflation_cells = int ((rover_radius / (resolution)) + saftey_margin)
        
        occupancy_grid = np.zeros(grid_size, dtype=np.float32)
        occupied_points = self.decode_octomap(octomap_msg)

        for point in occupied_points:
            x, y, z = point
            # Convert to grid indices
            grid_x = int((x - self.grid_origin[0]) / resolution)
            grid_y = int((y - self.grid_origin[1]) / resolution)
    
            # Clamp indices to valid bounds
            if 0 <= grid_x < grid_size[0] and 0 <= grid_y < grid_size[1]:
                if 0 <= z <= 50:  # Height threshold -< this should be 0.2<z<1.5!!!
                    #normalized_cost = int((z - 0.2) / (1.5 - 0.2) * 100)
                    occupancy_grid[grid_x, grid_y] = 100000
                  
                    for dx in range(-inflation_cells, inflation_cells):
    
                            for dy in range(-inflation_cells-1, inflation_cells+1):
                                # new_x = grid_x + dx 
                                # new_y = grid_y + dy   
                                new_x = min(max(grid_x + dx, 0), grid_size[0] - 1)
                                new_y = min(max(grid_y + dy, 0), grid_size[1] - 1)   
                                print("inflation_cells", inflation_cells) 
                                occupancy_grid[new_x, new_y] = 100000  #np.inf  
           # rospy.loginfo(f"Grid position: ({grid_x}, {grid_y}) -> Cost: {occupancy_grid[grid_x, grid_y]}")
        return occupancy_grid

### A* start 

    def height_cost(self, current, neighbor): #this is g function for height
        current_height = self.occupancy_grid[current[0], current[1]]
        neighbor_height = self.occupancy_grid[neighbor[0], neighbor[1]]
        if current_height == -1 or neighbor_height == -1:  
            return 10000 
        if current_height == 10000 or neighbor_height==10000: 
            return 10000
        return abs(current_height - neighbor_height)
    

    def heuristic(self, node, goal): #h fucntion -> euclidean distance
        return np.linalg.norm(np.array(node) - np.array(goal))

    def reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path
    
    def a_star(self, start, goal):
        open_set = PriorityQueue()
        open_set.put((0, start))  # Put start with f_score = 0
        came_from = {}
        g_score = {start: 0}  # The cost of getting to the start node is 0
        f_score = {start: self.heuristic(start, goal)}  # Initial f_score based on heuristic

        while not open_set.empty():
            _, current = open_set.get()

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                height_cost = self.height_cost(current, neighbor)  # Get height-based cost
                tentative_g_score = g_score[current] + height_cost  # Add the height cost to the g_score
                if height_cost != 10000: 
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)  # f = g + h
                        open_set.put((f_score[neighbor], neighbor))
                else: 
                    print("H is actually infinity")
        rospy.logwarn("A* failed to find a path")
        return []
    
    def get_neighbors(self, node):
        """
        Get neighboring cells in the grid, avoiding obstacles (e.g., value of 100000).
        """
        neighbors = []
        x, y = node
        for dx, dy in [(-1, -1), (1, 1), (0, -1), (0, 1), (1, 0), (-1, 0)]:  # Check neighboring cells (up, down, left, right)
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.occupancy_grid.shape[0] and 0 <= ny < self.occupancy_grid.shape[1]:
                if self.occupancy_grid[nx, ny] !=10000:  
                    neighbors.append((nx, ny))
                else: 
                    print("OBJECT DETECTED")
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
        flattened_waypoints = [coord for waypoint in waypoints for coord in waypoint]
        msg.data = flattened_waypoints  # Set the data field with the flattened list
        self.astar_pub.publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            if self.occupancy_grid is None:
              #  print("STOPPED looking for path")
                rospy.logwarn("Waiting for occupancy grid...")
                self.rate.sleep()
                continue
           

            start =  (int(self.current_position_x), int(self.current_position_y))
           # print("THIS IS START", start)
            goal = (27, 3)  # change this!

            rospy.loginfo("Running A* algorithm...")
            path = self.a_star(start, goal)
            if path:
               # print("PATH Found", path)
                rospy.loginfo(f"Path found: {path}")
                current_pos = start
            
                for waypoint in path:
                    self.publish_velocity(current_pos, waypoint)
                    current_pos = waypoint
                    rospy.sleep(1 / self.update_rate) 
                self.publish_waypoints(path)
            self.rate.sleep()
            

if __name__ == "__main__":
    try:
        planner = OctoMapAStar()
        planner.run()
    except rospy.ROSInterruptException:
        pass

'''
    def odom_callback2(self, msg):
        """
        Callback to receive odometry or localization data.
        """
        # Extract robot's position from the Odometry message

        self.current_orientation = msg.pose.pose.orientation
        self.current_orientation_x =  msg.pose.pose.orientation.x
        self.current_orientation_y =  msg.pose.pose.orientation.y
        self.current_orientation_z =  msg.pose.pose.orientation.z
        self.current_orientation_w =  msg.pose.pose.orientation.w

        self.current_position = msg.pose.pose.position
        self.current_position_x = msg.pose.pose.position.x
        self.current_position_y = msg.pose.pose.position.y
        self.current_position_z = msg.pose.pose.position.z
        
      #  theta = get_theta_from_pose(msg.pose.position)
        
        (roll, pitch, yaw) = tf.euler_from_quaternion([self.current_orientation_x,
                                                       self.current_orientation_y,
                                                       self.current_orientation_z,
                                                       self.current_orientation_w])
        
        
        # Convert the updated Euler angles back to a quaternion
        q = tf.quaternion_from_euler(roll, pitch, yaw)
        print("ROLLLLLL", roll)

        # Update the current orientation
        self.current_orientation_x = q[0]
        self.current_orientation_y = q[1]
        self.current_orientation_z = q[2]
        self.current_orientation_w = q[3]

      
        rospy.loginfo("Current position: x=%f, y=%f, z=%f", 
                    self.current_position.x, 
                    self.current_position.y, 
                    self.current_position.z)
        
        rospy.loginfo("Current orientation: x=%f, y=%f, z=%f", 
                    self.current_orientation.x, 
                    self.current_orientation.y, 
                    self.current_orientation.z)
    
                    
   # def odom_callback3(self, msg):
    #     # Extract robot's position from the Odometry message
    #     self.current_position_x = msg.pose.pose.position.x
    #     self.current_position_y = msg.pose.pose.position.y
    #     self.current_position_z = msg.pose.pose.position.z

    #     # Extract robot's orientation (quaternion) from the Odometry message
    #     self.current_orientation_x = msg.pose.pose.orientation.x
    #     self.current_orientation_y = msg.pose.pose.orientation.y
    #     self.current_orientation_z = msg.pose.pose.orientation.z
    #     self.current_orientation_w = msg.pose.pose.orientation.w

    #     # Convert quaternion to Euler angles to get roll, pitch, and yaw (theta)
    #     (self.roll, self.pitch, self.yaw) = tf.euler_from_quaternion([
    #         self.current_orientation_x,
    #         self.current_orientation_y,
    #         self.current_orientation_z,
    #         self.current_orientation_w
    #     ])

    #     # Log the current position and orientation
    #     rospy.loginfo("Current position: x=%f, y=%f, z=%f", 
    #                 self.current_position_x, 
    #                 self.current_position_y, 
    #                 self.current_position_z)
        
    #     rospy.loginfo("Current orientation (roll, pitch, yaw): roll=%f, pitch=%f, yaw=%f", 
    #                 self.roll, self.pitch, self.yaw)
        
    #     # Publish the bounding box
    #     self.publish_bounding_box()

    
    def publish_bounding_box_scrap(self):
        #this was mainly for visualization.
        marker = Marker()
        inflation_radius = 0.3 
        marker.header.frame_id = "map"  # Adjust based on your TF setup
        marker.header.stamp = rospy.Time.now()
        marker.ns = "rover_bounding_box"
        marker.id = 0
        marker.type = Marker.CUBE  # Cube represents the bounding box
        marker.action = Marker.ADD

        # Define the size of the bounding box (Adjust these values)
        box_length = 1.0  # X dimension (meters)
        box_width = 0.8   # Y dimension (meters)
        box_height = 0.5  # Z dimension (meters)

        # Set the position of the bounding box (Center it around the rover)
        marker.pose.position.x = self.current_position_x
        marker.pose.position.y = self.current_position_y
        marker.pose.position.z = self.current_position_z + box_height / 2  # Center height
        print("ORIENTATION XXX", self.current_orientation_x*100)
    
        import tf.transformations as tf

        roll = self.current_orientation_x  # Assuming this is the roll angle (wrong if it's already a quaternion)
        pitch = self.current_orientation_y
        yaw = self.current_orientation_z

        # q = tf.quaternion_from_euler(roll, pitch, yaw) 

        # marker.pose.orientation.x = q[0] #self.current_orientation_x
        # marker.pose.orientation.y = q[1] #self.current_orientation_y
        # marker.pose.orientation.z = q[2] #self.current_orientation_z
        # marker.pose.orientation.w = 1.0

        # marker.pose.orientation.w = 1.0

        # Set the scale of the box
        marker.scale.x = box_length
        marker.scale.y = box_width 
        marker.scale.z = box_height

        # Set the color (RGBA)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5  #Transparency

        # Set lifetime (0 means it persists)
        marker.lifetime = rospy.Duration(0)

        # Publish the marker
        self.bounding_box_pub.publish(marker)
    


    def publish_bounding_box_with_no_rotation(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Adjust based on your TF setup
        marker.header.stamp = rospy.Time.now()
        marker.ns = "rover_bounding_box"
        marker.id = 0
        marker.type = Marker.LINE_STRIP  # Use LINE_STRIP to form a box
        marker.action = Marker.ADD

        # Set color and transparency
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully visible

        # Set scale (width of the bounding box lines)
        marker.scale.x = 0.1  # Thickness of lines

        # Define bounding box corners using precomputed wheel positions
        # corners = [
        #     Point(self.current_right_front_wheel_x, self.current_right_front_wheel_y, 0),  # Right Front
        #     Point(self.current_right_back_wheel_x, self.current_right_back_wheel_y, 0),    # Right Back
        #     Point(self.current_left_back_wheel_x, self.current_left_back_wheel_y, 0),      # Left Back
        #     Point(self.current_left_front_wheel_x, self.current_left_front_wheel_y, 0),    # Left Front
        #     Point(self.current_right_front_wheel_x, self.current_right_front_wheel_y, 0)   # Close the box
        # ]

        corners = [
        Point(self.current_position_x + 0.5, self.current_position_y + 0.5, 0),  # Right Front
        Point(self.current_position_x - 0.5, self.current_position_y + 0.5, 0),  # Right Back
        Point(self.current_position_x - 0.5, self.current_position_y - 0.5, 0),  # Left Back
        Point(self.current_position_x + 0.5, self.current_position_y - 0.5, 0),  # Left Front
        Point(self.current_position_x + 0.5, self.current_position_y + 0.5, 0)   # Close the box
    ]



        marker.points = corners  # Assign points to the marker

        # Publish the marker
        self.bounding_box_pub.publish(marker)

     def odom_callback4(self, msg):
        self.current_position_x = msg.pose.pose.position.x
        self.current_position_y = msg.pose.pose.position.y
        self.current_position_z = msg.pose.pose.position.z

        self.current_orientation_x= msg.pose.pose.orientation.x
        self.current_orientation_y=msg.pose.pose.orientation.y
        self.current_orintatoin_z=msg.pose.pose.orientation.z

        self.current_left_front_wheel_x=self.current_position_x + 4
        self.current_left_front_wheel_y=self.current_position_y - 4
        
        self.current_right_front_wheel_x = self.current_position_x +4 
        self.current_right_front_wheel_y = self.current_position_y +4

        self.current_left_back_wheel_x =  self.current_position_x - 4
        self.current_left_back_wheel_y = self.current_position_y -4

        self.current_right_back_wheel_x = self.current_position_x -4
        self.current_right_back_wheel_y= self.current_position_y +4

        #find this 

        # roll = self.current_orientation_x  # Assuming this is the roll angle (wrong if it's already a quaternion)
        # pitch = self.current_orientation_y
        # yaw = self.current_orientation_z

        # q = tf.quaternion_from_euler(self.current_orientation_x, self.current_orientation_y, self.current_orientation_z)
        
        

        # marker.pose.orientation.x = self.current_orientation_x
        # marker.pose.orientation.y = self.current_orientation_y
        # marker.pose.orientation.z = self.current_orientation_z
        # marker.pose.orientation.w = 1.0


        rospy.loginfo("Current position: x=%f, y=%f, z=%f", 
                    self.current_position_x, 
                    self.current_position_y, 
                    self.current_position_z)
        
        rospy.loginfo("Current position: x=%f, y=%f, z=%f", 
                    self.current_orientation_x, 
                    self.current_orientation_y, 
                    self.current_orientation_z)
        
        # Publish the bounding box
        self.publish_bounding_box()

        def publish_bounding_box4(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Adjust based on your TF setup
        marker.header.stamp = rospy.Time.now()
        marker.ns = "rover_bounding_box"
        marker.id = 0
        marker.type = Marker.LINE_STRIP  # LINE_STRIP connects the points
        marker.action = Marker.ADD

        # Set color and transparency
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully visible

        # Set scale (line thickness)
        marker.scale.x = 0.05  

        # Get the yaw angle (ensure it's in radians)
        theta = self.current_orientation_z  # If in degrees, convert: theta = math.radians(self.current_orientation_z)

        # Check initial angle
        print(f"Yaw (theta) at start: {theta:.2f} radians")

        # Define the **correct** corner positions (aligned with the axes initially)
        half_width = 0.5  
        original_corners = [
            (half_width,  half_width),   # Right Front
            (-half_width, half_width),   # Right Back
            (-half_width, -half_width),  # Left Back
            (half_width, -half_width),   # Left Front
            (half_width,  half_width)    # Close the box
        ]

        # Apply rotation and translation
        transformed_corners = []
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        for cx, cy in original_corners:
            # Rotate around the rover's center
            x = (cx * cos_theta - cy * sin_theta)  
            y = (cx * sin_theta + cy * cos_theta)  

            # Translate to the rover's actual position
            x += self.current_position_x  
            y += self.current_position_y  

            transformed_corners.append(Point(x, y, 0))

        marker.points = transformed_corners  # Assign transformed points to marker

        # Publish the marker
        self.bounding_box_pub.publish(marker)


        def publish_bounding_box_worksbutcouldbeimproved(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Adjust based on your TF setup
        marker.header.stamp = rospy.Time.now()
        marker.ns = "rover_bounding_box"
        marker.id = 0
        marker.type = Marker.LINE_STRIP  # Use LINE_STRIP to form a box
        marker.action = Marker.ADD

        # Set color and transparency
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully visible

        # Set scale (line thickness)
        marker.scale.x = 0.1  

        # Get the rover's yaw angle (assumes orientation is stored as Euler angles)
        yaw = self.current_orientation_z  # Assuming this is the yaw in radians

        # Define the unrotated corner positions relative to the rover
        half_width = 0.5
        half_length = 0.5

        corners = [
            (half_length, half_width),   # Right Front
            (-half_length, half_width),  # Right Back
            (-half_length, -half_width), # Left Back
            (half_length, -half_width),  # Left Front
            (half_length, half_width)    # Closing the box
        ]

        # Apply 2D rotation transformation
        rotated_corners = []
        for cx, cy in corners:
            x_rot = self.current_position_x + (cx * math.cos(yaw) - cy * math.sin(yaw))
            y_rot = self.current_position_y + (cx * math.sin(yaw) + cy * math.cos(yaw))
            rotated_corners.append(Point(x_rot, y_rot, 0))

        marker.points = rotated_corners  # Assign transformed points to marker

        # Publish the marker
        self.bounding_box_pub.publish(marker)
    
    def publish_bounding_box_trying_something_else(self):
    
        marker = Marker()
        marker.header.frame_id = "map"  # Adjust based on your TF setup
        marker.header.stamp = rospy.Time.now()
        marker.ns = "rover_bounding_box"
        marker.id = 0
        marker.type = Marker.LINE_STRIP  # Use LINE_STRIP to form a box
        marker.action = Marker.ADD

        # Set color and transparency
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully visible

        # Set scale (line thickness)
        marker.scale.x = 0.1  

        # Get the rover's yaw angle (assuming in radians)
        yaw = self.current_orientation_z  

        # Define the **fixed** corner positions relative to the rover's origin (center)
        half_width = 0.2
        half_length = 0.2

        # Define corners relative to the rover's body frame
        original_corners = [
            (half_length, half_width),   # Right Front
            (-half_length, half_width),  # Right Back
            (-half_length, -half_width), # Left Back
            (half_length, -half_width),  # Left Front
            (half_length, half_width)    # Closing the box
        ]

        # Apply rotation transformation to **original** (fixed) coordinates
        rotated_corners = []
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        for cx, cy in original_corners:
            x_rot = self.current_position_x + (cx * cos_yaw - cy * sin_yaw)
            y_rot = self.current_position_y + (cx * sin_yaw + cy * cos_yaw)
            rotated_corners.append(Point(x_rot, y_rot, 0))

        marker.points = rotated_corners  # Assign transformed points to marker

        # Publish the marker
        self.bounding_box_pub.publish(marker)

        def publish_bounding_box100(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Adjust based on your TF setup
        marker.header.stamp = rospy.Time.now()
        marker.ns = "rover_bounding_box"
        marker.id = 0
        marker.type = Marker.LINE_STRIP  # Use LINE_STRIP to form a box
        marker.action = Marker.ADD

        # Set color and transparency
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully visible

        # Set scale (line thickness)
        marker.scale.x = 0.05  
           # Ensure rover starts at (0,0) and is aligned with axes
        if not hasattr(self, "initialized"):
            self.current_position_x = 0.0
            self.current_position_y = 0.0
            self.current_orientation_z = 0.0  # Ensures no initial rotation
            self.initialized = True  # Set a flag so it only resets once


        # Get the rover's yaw angle (assuming in radians)
        theta = self.current_orientation_z  
        theta = math.atan2(math.sin(theta), math.cos(theta))

        # Define the **fixed** corner positions relative to the rover's center
        half_width = 0.5  # Adjusted from 4 to 0.5
        original_corners = [
            (half_width,  half_width),   # Right Front
            (-half_width, half_width),   # Right Back
            (-half_width, -half_width),  # Left Back
            (half_width, -half_width),   # Left Front
            (half_width,  half_width)    # Closing the box
        ]

        # Apply rotation and translation
        transformed_corners = []
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        for cx, cy in original_corners:
            x = cx * cos_theta - cy * sin_theta  # Rotate
            y = cx * sin_theta + cy * cos_theta  # Rotate

            x += self.current_position_x  # Translate
            y += self.current_position_y  # Translate

            transformed_corners.append(Point(x, y, 0))
        

        marker.points = transformed_corners  # Assign transformed points to marker

        # Publish the marker
        self.bounding_box_pub.publish(marker)
 
'''