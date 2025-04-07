

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