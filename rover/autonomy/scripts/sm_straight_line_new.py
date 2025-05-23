#!/usr/bin/python3

# This one is new. Won't use without testing. Adding detection furing straight line traversal

import rospy
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Bool
import math
import ar_detection_node as adn

import yaml
import os

file_path = os.path.join(os.path.dirname(__file__), "sm_config.yaml")

with open(file_path, "r") as f:
    sm_config = yaml.safe_load(f)

# File for the straight line traversal class for the state machine. The class is initialized with linear, angular velocities, and target passed
# through the state machine. When navigate function is called, it calls the move_to_target function which continuously calculates the target distance
# and heading to determine the angular and linear velocities. When the target distance is within the threshold it breaks out of the loop to return.
    
class StraightLineApproach:
    def __init__(self, lin_vel, ang_vel, targets):
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel
        self.targets = targets
        self.found = False
        self.abort_check = False
        self.x = -100000
        self.y = -100000
        self.heading = 0
        self.pose_subscriber = rospy.Subscriber('/pose', PoseStamped, self.pose_callback)
        self.target_subscriber = rospy.Subscriber('target', Float64MultiArray, self.target_callback)
        self.drive_publisher = rospy.Publisher('/drive', Twist, queue_size=10)
        self.aruco_found = False
        self.abort_sub = rospy.Subscriber("auto_abort_check", Bool, self.abort_callback)
        #new additions
        # self.aruco_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        self.aruco_sub = rospy.Subscriber("aruco_found", Bool, callback=self.detection_callback)
        self.aruco_sub = rospy.Subscriber("aruco_found", Bool, callback=self.aruco_detection_callback)
        self.mallet_sub = rospy.Subscriber('mallet_detected', Bool, callback=self.mallet_detection_callback)
        self.waterbottle_sub = rospy.Subscriber('waterbottle_detected', Bool, callback=self.waterbottle_detection_callback)
        self.message_pub = rospy.Publisher("gui_status", String, queue_size=10)
        # self.odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)

        #new additions
        # self.aruco_sub = rospy.Subscriber("aruco_found", Bool, callback=self.aruco_detection_callback)
        # self.mallet_sub = rospy.Subscriber('mallet_detected', Bool, callback=self.mallet_detection_callback)
        # self.waterbottle_sub = rospy.Subscriber('waterbottle_detected', Bool, callback=self.waterbottle_detection_callback)

    
    def pose_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.heading = self.to_euler_angles(msg.pose.orientation.w, msg.pose.orientation.x, 
                                            msg.pose.orientation.y, msg.pose.orientation.z)[2]

    def abort_callback(self,msg):
        self.abort_check = msg.data
        
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.heading = self.to_euler_angles(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)[2]

    def target_callback(self, msg):
        self.target_x = msg.data[0]
        self.target_y = msg.data[1]

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
    
       
    def aruco_detection_callback(self, data):
        time_now=time.time()
        if abs(self.timer-time_now) >5:
            self.timer=time_now
            self.count=0
        print("count,",self.count)
        # print("sm grid search_ data.data", data.data)
        if data.data:
            if self.count <= 4:
                self.count +=1
            else:
                self.aruco_found = data.data
                self.found_objects[self.state] = data.data
                self.count += 1
    
    def mallet_detection_callback(self, data):
        time_now=time.time()
        if abs(self.timer-time_now) >5:
            self.timer=time_now
            self.count=0
        if data.data:
            if self.count <= 4:
                self.count += 1
            else:
                self.mallet_found = data.data
                self.found_objects[self.state] = data.data
                self.count += 1

    def waterbottle_detection_callback(self, data):
        time_now=time.time()
        if abs(self.timer-time_now) >5:
            self.timer=time_now
            self.count=0
        if data.data:
            if self.count <= 4:
                self.count += 1
            else:
                    
                self.waterbottle_found = data.data
                self.found_objects[self.state] = data.data
                self.count += 1

    
    def detection_callback(self, data):
        self.found = data

    def move_to_target(self, target_x, target_y, state="Location Selection"): #navigate needs to take in a state value as well (FINISHIT)
        rate = rospy.Rate(50)
        kp = 0.5
        threshold = 0.5
        angle_threshold = 0.2
        
        obj = self.mallet_found or self.waterbottle_found
        
        mapping = {"AR1":self.aruco_found, 
                   "AR2":self.aruco_found,
                   "AR3":self.aruco_found,
                   "OBJ1":obj,
                   "OBJ2":obj}
        
        first_time=True

        while (not rospy.is_shutdown()) and (self.abort_check is False):
            msg = Twist()
            if target_x is None or target_y is None or self.x is None or self.y is None:
                continue
            obj = self.mallet_found or self.waterbottle_found
            mapping = {"AR1":self.aruco_found, 
                   "AR2":self.aruco_found,
                   "AR3":self.aruco_found,
                   "OBJ1":obj,
                   "OBJ2":obj}
            if mapping[state] is False:
                #nomral operation
                target_heading = math.atan2(target_y - self.y, target_x - self.x)
                target_distance = math.sqrt((target_x - self.x) ** 2 + (target_y - self.y) ** 2)
                # print(f"Current Position: ({self.x}, {self.y})")
                print("Target Heading:", math.degrees(target_heading), " Target Distance:", target_distance)
                angle_diff = target_heading - self.heading
                
                # print ( f"angle_diff: {angle_diff}")

                if angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                elif angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
                    
                # print (f"diff in heading: {angle_diff}", f"target_distance: {target_distance}")

                if target_distance < threshold:
                    msg.linear.x = 0
                    msg.angular.z = 0
                    self.drive_publisher.publish(msg)
                    print(f"Reached target: ({target_x}, {target_y})")
                    break

                if abs(angle_diff) <= angle_threshold:
                    msg.linear.x = self.lin_vel
                    msg.angular.z = 0
                else:
                    msg.linear.x = 0
                    msg.angular.z = angle_diff * kp
                    if abs(msg.angular.z) < 0.3:
                        msg.angular.z = 0.3 if msg.angular.z > 0 else -0.3
                self.drive_publisher.publish(msg)
            
            else: #if mapping[state] is True --> if the object is found
                print("mapping state is true!")
                print("IN HOMING")
                state="In Homing"
                self.message_pub.publish(state)
                # call homing
                # should publish that it is found
                # rospy.init_node('aruco_homing', anonymous=True) # change node name if needed
                pub = rospy.Publisher('drive', Twist, queue_size=10) # change topic name
                if state == "AR1" or state == "AR2" or state == "AR3":
                    # this sees which camera it is using and then uses the parameters accordingly.
                    if sm_config.get("realsense_detection"):
                        aimer = aruco_homing.AimerROS(640, 360, 700, 100, 100, sm_config.get("Ar_homing_lin_vel") , sm_config.get("Ar_homing_ang_vel")) # FOR ARUCO
                    else: 
                        aimer = aruco_homing.AimerROS(640, 360, 700, 100, 100, sm_config.get("Ar_homing_lin_vel") , sm_config.get("Ar_homing_ang_vel")) # FOR ARUCO
                    
                    rospy.Subscriber('aruco_node/bbox', Float64MultiArray, callback=aimer.rosUpdate) # change topic name
                    print (sm_config.get("Ar_homing_lin_vel"),sm_config.get("Ar_homing_ang_vel"))
                elif state == "OBJ1" or state == "OBJ2":
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
                            if first_time:
                                first_time=False
                                initial_time=time.time()
                            
                            # faking it til you makin it
                            while abs(initial_time-time.time()) < 0.7:
                                msg.linear.x=self.lin_vel
                                pub.publish(msg)
                                print("final homing movement",abs(initial_time-time.time()) )
                                rate.sleep()
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                            pub.publish(twist)
                            return
                            
                        # Normal homing behavior
                        if aimer.angular_v == 1:
                            twist.angular.z = float(aimer.max_angular_v)
                            print ("first if",aimer.max_angular_v)
                            twist.linear.x = 0.0
                        elif aimer.angular_v == -1:
                            twist.angular.z = float(-aimer.max_angular_v)
                            twist.linear.x = 0.0
                        elif aimer.linear_v == 1:
                            print ("second check",aimer.max_linear_v)
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
            
            self.drive_publisher.publish(msg)
            rate.sleep()

    def navigate(self, state="Location Selection"): #navigate needs to take in a state value as well
        for target_x, target_y in self.targets:
            print(f"Moving towards target: ({target_x}, {target_y})")
            self.move_to_target(target_x, target_y)
            if self.abort_check:
                self.abort_check = False
                break
            rospy.sleep(1)

if __name__ == '__main__':
    targets = [(9, 2)]  # Define multiple target points
    try:
        rospy.init_node('straight_line_approach_node')
        approach = StraightLineApproach(1.5, 0.5, targets)
        approach.navigate()
    except rospy.ROSInterruptException:
        pass

    gs = GridSearch(4, 4, 1, x, y)  # define multiple target points here: cartesian
    target = gs.square_target()
    print(target)
    try:
        straight_line_approach(1, 0.5, target) #LOOK HERE
    except rospy.ROSInterruptException:
        pass