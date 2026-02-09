#!/usr/bin/python3

# This one is new because it can home during straight line traversal. 

import rclpy
from rclpy.node import Node
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Bool, String
from std_msgs.msg import String as StdString
from rover.msg import MissionState
import math
import ar_detection_node as adn
import aruco_homing as aruco_homing
import ar_detection_node as ar_detect
from rclpy.executors import MultiThreadedExecutor
import threading
import time

import yaml
import os

file_path = os.path.join(os.path.dirname(__file__), "sm_config.yaml")

with open(file_path, "r") as f:
    sm_config = yaml.safe_load(f)

# File for the straight line traversal class for the state machine. The class is initialized with linear, angular velocities, and target passed
# through the state machine. When navigate function is called, it calls the move_to_target function which continuously calculates the target distance
# and heading to determine the angular and linear velocities. When the target distance is within the threshold it breaks out of the loop to return.
    
class reattemptTarget(Node):
    def __init__(self):
        super().__init__('straight_line_approach_new')
        self.lin_vel = sm_config.get("GS_Traversal_lin_vel") 
        self.ang_vel = sm_config.get("GS_Traversal_ang_vel") #Chosen this way for now
        self.targets = None 
        self.found = False
        self.timer = 0
        self.state=None
        self.abort_check = False
        self.x = 0
        self.y = 0
        self.done_early = False
        self.start_looking = False
        self.heading = 0
        self.start_x=0 #new
        self.start_y=0 #new
        
        # self.pose_subscriber = rospy.Subscriber('/pose', PoseStamped, self.pose_callback)
        # self.target_subscriber = rospy.Subscriber('target', Float64MultiArray, self.target_callback)
        # self.drive_publisher = rospy.Publisher('/drive', Twist, queue_size=10)
        
        self.aruco_found = False
        self.count=0
        # self.abort_sub = rospy.Subscriber("auto_abort_check", Bool, self.abort_callback)
        if sm_config.get("realsense_detection"): #moved here
            aimer = aruco_homing.AimerROS(640, 360, 2500, 100, 100, sm_config.get("Ar_homing_lin_vel") , sm_config.get("Ar_homing_ang_vel")) # FOR ARUCO
        else: 
            aimer = aruco_homing.AimerROS(640, 360, 700, 100, 100, sm_config.get("Ar_homing_lin_vel") , sm_config.get("Ar_homing_ang_vel")) # FOR ARUCO
        
        #new additions
        # self.aruco_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        # self.aruco_sub = rospy.Subscriber("aruco_found", Bool, callback=self.aruco_detection_callback)
        # self.mallet_sub = rospy.Subscriber('mallet_detected', Bool, callback=self.mallet_detection_callback)
        # self.waterbottle_sub = rospy.Subscriber('waterbottle_detected', Bool, callback=self.waterbottle_detection_callback)
        # self.message_pub = rospy.Publisher("gui_status", String, queue_size=10)
        # self.done_early = rospy.Publisher("done_early", Bool, queue_size=10)
        self.pose_subscriber = self.create_subscription(PoseStamped, '/pose', self.pose_callback, 10)
        self.target_subscriber = self.create_subscription(Float64MultiArray, 'target', self.target_callback, 10)
        self.odom_subscriber =self.create_subscription(Odometry, '/rtabmap/odom', self.odom_callback, 10)
        self.aruco_sub = self.create_subscription(Bool, 'aruco_found', self.aruco_detection_callback, 10)
        self.mallet_sub = self.create_subscription(Bool, 'mallet_detected', self.mallet_detection_callback, 10)
        self.waterbottle_sub = self.create_subscription(Bool, 'waterbottle_detected', self.waterbottle_detection_callback, 10)
        self.pub = self.create_subscription(MissionState,'mission_state',self.feedback_callback, 10)
        self.abort_sub = self.create_subscription(Bool, "auto_abort_check", self.abort_callback, 10)
        self.aruco_bbox_sub= self.create_subscription(Float64MultiArray, 'aruco_node/bbox', aimer.rosUpdate, 10) #moved here
        
        self.message_pub = self.create_publisher(String, "gui_status", 10)
        self.done_early = self.create_publisher(Bool, "done_early", 10)
        self.pub = self.create_publisher(MissionState, 'mission_state', 10) #new for integrating with the state machine
        self.drive_publisher = self.create_publisher(Twist, '/drive', 10)
        
        # self.odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        # self.odom_sub = self.create_subscription(Odometry, '/rtabmap/odom', self.odom_callback, 10)
        self.found_objects = {"AR_REC":False, 
                   "OBJ_REC":False
                   }
        self.aruco_found = False
        self.mallet_found = False
        self.waterbottle_found = False

        self._nav_thread = None  #new

        #new additions
        #  elf.aruco_sub = rospy.Subscriber("aruco_found", Bool, callback=self.aruco_detection_callback)
        # self.mallet_sub = rospy.Subscriber('mallet_detected', Bool, callback=self.mallet_detection_callback)
        # self.waterbottle_sub = rospy.Subscriber('waterbottle_detected', Bool, callback=self.waterbottle_detection_callback)
    
    def feedback_callback(self, msg):
        print("in Reattempt feedback callback, msg.state:", msg.state)
        if msg.state == "START_REATTEMPT": #Will change later
            self.active = True
            self.state = msg.current_state
            i = 0
            while(i < len(msg.targets)):
                self.targets.append((msg.targets[i], msg.targets[i+1]))
                i+=2
            self.get_logger().info("Reattempt behavior ACTIVE")
            self.start_x = msg.current_goal.pose.position.x
            self.start_y = msg.current_goal.pose.position.y
            if self._nav_thread is None or not self._nav_thread.is_alive():
                self._nav_thread = threading.Thread(target=self.navigate, daemon=True)
                self._nav_thread.start()
        # elif msg.state in ("HOMING_DONE", "HOMING_SUCCESS", "HOMING_FAILED"):
        #     self.homing_status = msg.state
        else:
            self.active = False

    
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
        try:
            if self.start_looking:
                print("in aruco detection callback", self.count)
                time_now=time.time()
                if abs(self.timer-time_now) > 5:
                    self.timer = time_now
                    self.count = 0
                print("count,",self.count)
                # print("sm grid search_ data.data", data.data)
                if data.data:
                    if self.count <= 4:
                        self.count +=1
                    else:
                        self.aruco_found = data.data
                        if self.state in self.found_objects:
                            self.found_objects[self.state] = data.data
                        else:  
                            self.get_logger().warn(f"aruco_detection_callback: state '{self.state}' not in found_objects")
                        self.count += 1
        except Exception as e:
            self.get_logger().error(f"Exception in aruco_detection_callback: {e}")
    
    def mallet_detection_callback(self, data):
        try:
            time_now=time.time()
            if abs(self.timer-time_now) >5:
                self.timer=time_now
                self.count=0
            if data.data:
                if self.count <= 4:
                    self.count += 1
                else:
                    self.mallet_found = data.data
                    if self.state in self.found_objects:
                        self.found_objects[self.state] = data.data
                    else:
                        self.get_logger().warn(f"mallet_detection_callback: state '{self.state}' not in found_objects")
                    self.count += 1
        except Exception as e:
            self.get_logger().error(f"Exception in mallet_detection_callback: {e}")

    def waterbottle_detection_callback(self, data):
        print("in waterbottle detection callback")
        try:
            time_now=time.time()
            if abs(self.timer-time_now) >5:
                self.timer=time_now
                self.count=0
            if data.data:
                if self.count <= 4:
                    self.count += 1
                else:
                    self.waterbottle_found = data.data
                    if self.state in self.found_objects:
                        self.found_objects[self.state] = data.data
                    else:
                        self.get_logger().warn(f"waterbottle_detection_callback: state '{self.state}' not in found_objects")         
                    self.count += 1
        except Exception as e:
            self.get_logger().error(f"Exception in waterbottle_detection_callback: {e}")
    
    def detection_callback(self, data):
        self.found = data
        
    def spin(self,state):
        # method one on angle
        # backup to timing too in case angles fails
        startTime = time.time()
        startAngle = self.heading[2]
        msg = Twist()
        while (rclpy.ok()):
            if self.heading[2]-startAngle<20 or time.time()-startTime>15:
                msg.angular.x = 0.0
                msg.angular.z = 0.0
                self.drive_publisher.publish(msg)
                return False
            
            if self.mapping[state] ==True:
                msg.angular.x = 0.0
                msg.angular.z = 0.0
                self.drive_publisher.publish(msg)
                return True
            msg.angular.x = 0.0
            msg.angular.z = 0.5
            self.drive_publisher.publish(msg)
            
        
            
            
                
            
    def move_to_target(self, target_x, target_y, state): #navigate needs to take in a state value as well (FINISHIT)
        
        kp = 0.5
        threshold = 0.5
        angle_threshold = 0.2
        
        obj = self.mallet_found or self.waterbottle_found
        
        self.mapping = {"AR1":self.aruco_found, 
                   "AR2":self.aruco_found,
                   "AR3":self.aruco_found,
                   "OBJ1":obj,
                   "OBJ2":obj}
        
        first_time=True
        

        while (rclpy.ok()) and (self.abort_check is False):
            msg = Twist()
            if target_x is None or target_y is None or self.x is None or self.y is None:
                print("tagret x, taget y ,self.x, self.y is none ")
                continue
            
            if abs(self.x-target_x) < 25 or abs(self.y-target_y) < 25: #Where are these numbers coming from?
                self.start_looking=True
                
            obj = self.mallet_found or self.waterbottle_found
            self.mapping = {"AR_REC":self.aruco_found, 
                   "OBJ_REC":obj}
            print("state is mapping[state]", self.mapping[state], state)
            if self.mapping[state] is False:
                #nomral operation
                target_heading = math.atan2(target_y - self.y, target_x - self.x)
                target_distance = math.sqrt((target_x - self.x) ** 2 + (target_y - self.y) ** 2)
                print("Target Heading:", math.degrees(target_heading), " Target Distance:", target_distance)
                # print(f"Current Position: ({self.x}, {self.y})")
                #print("Target Heading:", math.degrees(target_heading), " Target Distance:", target_distance)
                angle_diff = target_heading - self.heading
                
                # print ( f"angle_diff: {angle_diff}")

                if angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                elif angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
                    
                # print (f"diff in heading: {angle_diff}", f"target_distance: {target_distance}")

                if target_distance < threshold:
                    msg.linear.x = float(0)
                    msg.angular.z = float(0)
                    self.drive_publisher.publish(msg)
                    print(f"Reached target: ({target_x}, {target_y})")
                    if(self.spin(state)):
                        continue
                    else:
                        pass # if spining doesn't locate continue moving to target

                if abs(angle_diff) <= angle_threshold:
                    msg.linear.x = self.lin_vel
                    msg.angular.z = float(0)
                else:
                    msg.linear.x = float(0)
                    msg.angular.z = angle_diff * kp
                    if abs(msg.angular.z) < 0.3:
                        msg.angular.z = 0.3 if msg.angular.z > 0 else -0.3
                
            
            elif self.mapping[state]==True: #if mapping[state] is True --> if the object is found
                print("mapping state is true!")
                print("IN HOMING")
                message="In Homing"
                self.message_pub.publish(StdString(data=message))
                # call homing
                # should publish that it is found
                # rospy.init_node('aruco_homing', anonymous=True) # change node name if needed
                # pub = rospy.Publisher('drive', Twist, queue_size=10) # change topic name
                if state == "AR_REC":
                    if self.aruco_bbox_sub is None:
                        # self.aruco_bbox_sub= self.create_subscription(Float64MultiArray, 'aruco_node/bbox', aimer.rosUpdate, 10) 
                        print(sm_config.get("Ar_homing_lin_vel"),sm_config.get("Ar_homing_ang_vel"))
                    #print (sm_config.get("Ar_homing_lin_vel"),sm_config.get("Ar_homing_ang_vel"))
                elif state == "OBJ_REC":
                    aimer = aruco_homing.AimerROS(640, 360, 1450, 100, 200, sm_config.get("Obj_homing_lin_vel"), sm_config.get("Obj_homing_ang_vel")) # FOR WATER BOTTLE
                    #print (sm_config.get("Obj_homing_lin_vel"),sm_config.get("Obj_homing_ang_vel"))
                    print (sm_config.get("Obj_homing_lin_vel"),sm_config.get("Obj_homing_ang_vel"))
                 #this code needs to be adjusted
                
                # Wait a bit for initial detection
                for i in range(50):
                    time.sleep(0.1)
                
                # Add variables for tracking detection memory
                last_detection_time = time.time()
                detection_memory_duration = 2.0  # 2 seconds of memory
                detection_active = False
                
                while (rclpy.ok()) and (self.abort_check is False):
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
                                self.drive_publisher.publish(msg)
                                print("final homing movement",abs(initial_time-time.time()) )
                                time.sleep(0.1)
                            twist.linear.x = 0.0
                            msgg=True
                            self.done_early.publish(msgg)
                            twist.angular.z = 0.0
                            self.drive_publisher.publish(twist)
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
                        
                        last_detectionp_linear_velocity = float(twist.linear.x) #this line to be used when we are scaling the speed.
                        last_detectionp_angular_velocity = float(twist.angular.z) #this line to be used when we are scaling the speed.

                    else:

                        if detection_active and time.time() - last_detection_time < detection_memory_duration:
                            twist.linear.x = last_detectionp_linear_velocity * -1
                            twist.angular.z = last_detectionp_angular_velocity * -1
                            print("Going back or turning the angle back when no detection and still in within the memory duration") 
                        else:
                            print("Detection lost and memory expired, returning to grid search")
                            detection_active = False
                            break
                    
                    self.drive_publisher.publish(twist)
                    time.sleep(1/50) #Was 0.1 in grid search
              
                break
            
            self.drive_publisher.publish(msg)
            time.sleep(1/50) #Was 0.1 in grid search

    def navigate(self): #navigate needs to take in a state value as well
        msg = MissionState()
        for target_x, target_y in self.targets:
            if self.found_objects[self.state]: #should be one of aruco, mallet, waterbottle
                print(f"Object detected during navigation: {self.found_objects[self.state]}")
                msg.state="OBJ_FOUND" #Can be changed
                self.pub.publish(msg)
                return True
            print(f"Moving towards target: ({target_x}, {target_y})")
            self.move_to_target(target_x, target_y, self.state)
            if self.abort_check:
                self.abort_check = False
                break
            time.sleep(1)
        
        msg = MissionState()
        if self.found_objects[self.state] and self.abort_check is False:
            msg.state="OBJ_FOUND"
            self.pub.publish(msg)
            return True
        msg.state="OBJ_NOT_FOUND"
        self.pub.publish(msg)
        return False 

if __name__ == '__main__':
    import rclpy
    rclpy.init()
    executor = MultiThreadedExecutor()
    executor.add_node(rea)
    rea= reattemptTarget() 
    try:
        executor.spin(rea)
    finally:
        rea.destroy_node()
        rclpy.shutdown()


    
    """ targets = [(9, 2)]  # Define multiple target points
    rclpy.init()
    try:
        
        approach = reattemptTarget(1.5, 0.5, targets, 'AR1')
        approach.navigate()
    except rclpy.exceptions.ROSInterruptException:
        pass

    gs = GridSearch(4, 4, 1, x, y)  # define multiple target points here: cartesian
    target = gs.square_target()
    print(target)
    try:
        straight_line_approach(1, 0.5, target) #LOOK HERE
    except rclpy.exceptions.ROSInterruptException:
        pass """