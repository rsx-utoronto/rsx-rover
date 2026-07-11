#!/usr/bin/python3

from calian_gnss_ros2_msg import msg
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float64MultiArray, Bool, String
from rover.msg import MissionState
from std_msgs.msg import String as StdString
import math
# import aruco_homing as aruco_homing
import aruco_homing_improve as aruco_homing
from rclpy.executors import MultiThreadedExecutor
import yaml
import os
import threading
import time

file_path = os.path.join(os.path.dirname(__file__), "sm_config.yaml")

with open(file_path, "r") as f:
    sm_config = yaml.safe_load(f)

# File for the grid search class. Class is initialized with linear, angular velocities, and the target location sent through
# the state machine. When the function square_target is called, it generates grid search targets in a spiral to navigate to. When 
# navigate function is called it calls the move_to_target function on the generated square targets, tries to navigate to each of them. 
# When aruco, waterbottle, or mallet callback functions detect an object in the corresponding state, it calls aruco homing
# to navigate to the detected item to get close enough
    
class GS_Traversal(Node):
    def __init__(self):
        super().__init__('gs_traversal_node')
        self.lin_vel = sm_config.get("GS_Traversal_lin_vel") 
        self.ang_vel = sm_config.get("GS_Traversal_ang_vel")
        self.targets = None
        self.found = False 
        self.abort_check = False
        self.x = 0
        self.timer=0
        self.y = 0
        self.heading = 0
        self.heading = math.pi
        self.state = None
        self.current_state = None
        self.count = 0
        self.timer=0
        self.w = sm_config.get("AR_grid_search_w")
        self.h = sm_config.get("AR_grid_search_h")
        self.tol = sm_config.get("AR_grid_search_tol")
        self.ar1_w = sm_config.get("AR1_grid_search_w")
        self.ar1_h = sm_config.get("AR1_grid_search_h")
        self.ar1_tol = sm_config.get("AR1_grid_search_tol")
        self.ar2_w = sm_config.get("AR2_grid_search_w")
        self.ar2_h = sm_config.get("AR2_grid_search_h")
        self.ar2_tol = sm_config.get("AR2_grid_search_tol")
        self.obj1_w = sm_config.get("OBJ1_grid_search_w")
        self.obj1_h = sm_config.get("OBJ1_grid_search_h")
        self.obj1_tol = sm_config.get("OBJ1_grid_search_tol")
        self.obj2_w = sm_config.get("OBJ2_grid_search_w")
        self.obj2_h = sm_config.get("OBJ2_grid_search_h")
        self.obj2_tol = sm_config.get("OBJ2_grid_search_tol")
        self.obj3_w = sm_config.get("OBJ3_grid_search_w")
        self.obj3_h = sm_config.get("OBJ3_grid_search_h")
        self.obj3_tol = sm_config.get("OBJ3_grid_search_tol")
        self.start_x=0
        self.start_y=0

        # store whether each object has been found at least once. 
        self.aruco_found = False
        self.mallet_found = False
        self.waterbottle_found = False
        self.hammer_found = False

        self.initial_aruco_found = False
        self.initial_mallet_found = False
        self.initial_waterbottle_found = False
        self.initial_pick_hammer_found = False

        self.homing_status = None
        # Define dictionary for storing found objects
        self.found_objects = {"AR1":False, 
                   "AR2":False,
                   "OBJ1":False,
                   "OBJ2":False, 
                   "OBJ3":False}
        
        self.odom_subscriber = self.create_subscription(Odometry, '/rtabmap/odom', self.odom_callback, 10)  # modified to use rclpy
        self.pose_subscriber = self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)    
        self.target_subscriber = self.create_subscription(Float64MultiArray, 'target', self.target_callback, 10)
        self.drive_publisher = self.create_publisher(Twist, '/drive', 10)  # modified to use rclpy
        self.aruco_sub = self.create_subscription(Bool, "aruco_found", self.aruco_detection_callback, 10)
        self.mallet_sub = self.create_subscription(Bool, 'mallet_detected', self.mallet_detection_callback, 10)
        self.waterbottle_sub = self.create_subscription(Bool, 'waterbottle_detected', self.waterbottle_detection_callback, 10)
        self.hammer_sub = self.create_subscription(Bool, 'pick_hammer_detected', self.hammer_detection_callback, 10)
        self.abort_sub = self.create_subscription(Bool, "/auto_abort_check", self.abort_callback, 10)
        self.message_pub = self.create_publisher(String, "gui_status", 10)
        # the aimer declarations here are very weird. Changed into self.aimer and then initialize them.

        aruco_area = 2500 if sm_config.get("realsense_detection") else 750
        self.aimer = aruco_homing.AimerROS(640, 360, aruco_area, 100, 100, sm_config.get("Ar_homing_lin_vel") , sm_config.get("Ar_homing_ang_vel")) # FOR ARUCO

        
        # if sm_config.get("realsense_detection"):
        #     aimer = aruco_homing.AimerROS(640, 360, 2500, 100, 100, sm_config.get("Ar_homing_lin_vel") , sm_config.get("Ar_homing_ang_vel")) # FOR ARUCO
        # else: 
        #     aimer = aruco_homing.AimerROS(640, 360, 700, 100, 100, sm_config.get("Ar_homing_lin_vel") , sm_config.get("Ar_homing_ang_vel")) # FOR ARUCO
        
        self.aruco_bbox_sub= self.create_subscription(Float64MultiArray, 'aruco_node/bbox', self.aimer.rosUpdate, 10) 
        # self.object_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        self.pub = self.create_publisher(MissionState, 'mission_state', 10)
        self.create_subscription(MissionState,'mission_state',self.feedback_callback, 10)
        self._nav_thread = None
        # self.aimer_aruco=None
        # self.aimer_object=None
        self.aruco_bbox_sub=None
        self.object_bbox_sub=None
        self.avoidance_in_progress = False
        
        # for astar threading
        self.astar_event = threading.Event()
        self.astar_result = None
        

    def feedback_callback(self, msg):
        if msg.state in ["START_GS_TRAV", "START_GS_TRAV_AVOIDANCE", "GS_WAYPOINT_REACHED"]:
            self.state = msg.state
            if msg.current_state:
                self.current_state = msg.current_state
            self.start_x = self.x
            self.start_y = self.y
            # self.get_logger().info(f"Grid Search behavior ACTIVE {self.state}")
            
            if msg.state == "START_GS_TRAV_AVOIDANCE" or msg.state == "GS_WAYPOINT_REACHED":
                self.avoidance_in_progress = True
            else:
                self.avoidance_in_progress = False
            if self._nav_thread is None or not self._nav_thread.is_alive():
                # self.get_logger().info("Grid Search with Avoidance behavior ACTIVE")
                self._nav_thread = threading.Thread(target=self.navigate, daemon=True)
                self._nav_thread.start()
      
       
    def pose_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.heading = self.to_euler_angles(msg.pose.orientation.w, msg.pose.orientation.x, 
                                            msg.pose.orientation.y, msg.pose.orientation.z)[2]
        self.heading2 = self.heading + math.pi

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.heading = self.to_euler_angles(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)[2]
        self.heading2 = self.heading + math.pi

    def abort_callback(self,msg):
        self.abort_check = msg.data

    def target_callback(self, msg):
        self.target_x = msg.data[0]
        self.target_y = msg.data[1]
    
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
    
    def aruco_detection_callback(self, data):
        try:
            time_now=time.time()
            if abs(self.timer-time_now) >5:
                self.timer=time_now
                self.count=0
            print("count,",self.count)
            if data.data:
                if self.count <= 4:
                    self.count += 1
                else:
                    self.aruco_found = data.data
                    if self.current_state in self.found_objects:
                        self.found_objects[self.current_state] = data.data
                    else:
                        self.get_logger().warn(f"aruco_detection_callback: state '{self.current_state}' not in found_objects")
                    self.count += 1
        except Exception as e:
            self.get_logger().error(f"Exception in aruco_detection_callback: {e}")
    
    def mallet_detection_callback(self, data):
        # print("in mallet detection callback")
        try:
            if self.current_state in self.found_objects and self.found_objects[self.current_state]:
                return
            time_now=time.time()
            if abs(self.timer-time_now) >5:
                self.timer=time_now
                self.count=0
            if data.data:
                if self.count <= 4:
                    self.count += 1
                else:
                    self.mallet_found = data.data
                    if self.current_state in self.found_objects:
                        self.found_objects[self.current_state] = data.data
                        if self.initial_mallet_found == False:
                            self.initial_mallet_found = True
                            msg = MissionState()
                            msg.state = "OBJ_FOUND"
                            self.pub.publish(msg) # function called pub. Maybe use the same name for state pub/sub callbacks for all files? 
                            drive_msg = Twist()
                            drive_msg.linear.x = float(0)
                            drive_msg.angular.z = float(0)
                            self.drive_publisher.publish(drive_msg)
                    else:
                        self.get_logger().warn(f"mallet_detection_callback: state '{self.current_state}' not in found_objects")
                    self.count += 1
        except Exception as e:
            self.get_logger().error(f"Exception in mallet_detection_callback: {e}")

    def waterbottle_detection_callback(self, data):
        if self.current_state in self.found_objects and self.found_objects[self.current_state]:
                return
        # print("in waterbottle detection callback")
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
                    if self.current_state in self.found_objects:
                        self.found_objects[self.current_state] = data.data

                        if self.initial_waterbottle_found == False:
                            self.initial_waterbottle_found = True
                            msg = MissionState()
                            msg.state = "OBJ_FOUND"
                            self.pub.publish(msg)
                            drive_msg = Twist()
                            drive_msg.linear.x = float(0)
                            drive_msg.angular.z = float(0)
                            self.drive_publisher.publish(drive_msg)
                    else:
                        self.get_logger().warn(f"waterbottle_detection_callback: state '{self.current_state}' not in found_objects")
                    self.count += 1
        except Exception as e:
            self.get_logger().error(f"Exception in waterbottle_detection_callback: {e}")

    def hammer_detection_callback(self, data):
        if self.current_state in self.found_objects and self.found_objects[self.current_state]:
                return
        # print("in hammer detection callback", self.hammer_found)
        
        try:
            time_now=time.time()
            if abs(self.timer-time_now) >5:
                self.timer=time_now
                self.count=0
            if data.data:
                if self.count <= 4:
                    self.count += 1
                else:
                    self.hammer_found = data.data
                    if self.current_state in self.found_objects:
                        self.found_objects[self.current_state] = data.data
                        if self.initial_pick_hammer_found == False:
                            self.initial_pick_hammer_found = True
                            msg = MissionState()
                            msg.state = "OBJ_FOUND"
                            self.pub.publish(msg)
                            drive_msg = Twist()
                            drive_msg.linear.x = float(0)
                            drive_msg.angular.z = float(0)
                            self.drive_publisher.publish(drive_msg)
                    else:
                        self.get_logger().warn(f"hammer_detection_callback: state '{self.current_state}' not in found_objects")
                    self.count += 1
        except Exception as e:
            self.get_logger().error(f"Exception in hammer_detection_callback: {e}")

    def square_target(self):
        print("Generating square grid search targets...")
        dx, dy = 0, 1
        # use local copies so we don't mutate the node's starting point
        sx, sy = float(self.start_x), float(self.start_y)
        targets = [(sx, sy)]
        step = 1
        out_of_bounds = False
        if self.current_state == "AR1":
            w = self.ar1_w
            h = self.ar1_h
            tol = self.ar1_tol
        elif self.current_state == "AR2":
            w = self.ar2_w
            h = self.ar2_h
            tol = self.ar2_tol
        elif self.current_state == "OBJ1":
            w = self.obj1_w
            h = self.obj1_h
            tol = self.obj1_tol
        elif self.current_state == "OBJ2":
            w = self.obj2_w
            h = self.obj2_h
            tol = self.obj2_tol
        elif self.current_state == "OBJ3":
            w = self.obj3_w
            h = self.obj3_h
            tol = self.obj3_tol

        while len(targets) < int(w)**2 - 1:
            for _ in range(2):
                for _ in range(step):
                    if step * float(tol) > float(w):
                        out_of_bounds = True
                        break
                    sx, sy = sx + (tol * dx), sy + (tol * dy)
                    targets.append((sx, sy))
                dx, dy = -dy, dx
                if out_of_bounds:
                    break
            step += 1
            if out_of_bounds:
                break

        print("these are the final targets", targets)
        return targets
    
    def move_to_target(self, target_x, target_y, current_state): #navigate needs to take in a state value as well (FINISHIT)
        kp = 0.5
        threshold = 0.7
        angle_threshold = 0.2

        # logic here might not be ideal - could be some weird scenario where the state is not one of these despite 
        # only being called when grid_search is required
        obj = self.mallet_found or self.waterbottle_found or self.hammer_found
        
        self.mapping = {"AR1":self.aruco_found, 
                   "AR2":self.aruco_found,
                   "OBJ1":obj,
                   "OBJ2":obj, 
                   "OBJ3":obj}
        first_time=True
        
        

        while (rclpy.ok()) and (self.abort_check is False):
            # self.get_logger().info(f"GRID SEARCH Moving to target: ({target_x}, {target_y}) for state {current_state}")
            # Allow ROS callbacks to process during the tight loop
         
            
            obj = self.mallet_found or self.waterbottle_found or self.hammer_found
            mapping = {"AR1":self.aruco_found, 
                   "AR2":self.aruco_found,
                   "OBJ1":obj,
                   "OBJ2":obj, 
                   "OBJ3":obj}
            msg = Twist()
            if mapping[current_state] is False: # while not detected
                # self.get_logger().info(f"GRID SEARCH Moving to target: ({target_x}, {target_y}) for state {current_state}")
                # normal operations
                if target_x is None or target_y is None or self.x is None or self.y is None:
                    print("target_x, target_y ,self.x, self.y is none.")
                    continue

                target_heading = math.atan2(target_y - self.y, target_x - self.x)
                target_distance = math.sqrt((target_x - self.x) ** 2 + (target_y - self.y) ** 2)
                # print("Target Heading:", math.degrees(target_heading), " Target Distance:", target_distance)
                angle_diff = target_heading - self.heading

                # Accounting for the angle wraparound 
                if angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                elif angle_diff < -math.pi:
                    angle_diff += 2 * math.pi

                if target_distance < threshold:
                    self.get_logger().info(f"GRID SEARCH Reached target: ({target_x}, {target_y}) for state {current_state}")
                    msg.linear.x = float(0)
                    msg.angular.z = float(0)
                    self.drive_publisher.publish(msg)
                    print(f"Reached target: ({target_x}, {target_y})")
                    break 

                # Stops rotating once angle diff is less than threshold
                if abs(angle_diff) <= angle_threshold:
                    self.get_logger().info(f"GRID SEARCH Heading aligned with target: ({target_x}, {target_y}) for state {current_state}")
                    print("approaching x", self.lin_vel)
                    msg.linear.x = float(self.lin_vel) 
                    msg.angular.z = float(0)
                    self.drive_publisher.publish(msg)
                else:
                    print("adjusting angle")
                    self.get_logger().info(f"GRID SEARCH Adjusting heading to target: ({target_x}, {target_y}) for state {current_state}")
                    msg.linear.x = float(0)
                    msg.angular.z = float(angle_diff * kp)
                    if abs(msg.angular.z) < 0.3:
                        msg.angular.z = math.copysign(1, msg.angular.z) * float(0.3) #0.3 if msg.angular.z > 0 else -0.3 CHANGE WHEN TESTING OUSTIDE
                    self.drive_publisher.publish(msg)

            elif self.aruco_found: #if mapping[current_state] is True and is an aruco tag is found 
                print("IN HOMING! mapping state is true!")
                message="In Homing"
                # self.message_pub.publish(message)
                self.message_pub.publish(StdString(data=message))

                if self.aruco_bbox_sub is None:
                        # self.aruco_bbox_sub= self.create_subscription(Float64MultiArray, 'aruco_node/bbox', aimer.rosUpdate, 10) 
                    print (sm_config.get("Ar_homing_lin_vel"),sm_config.get("Ar_homing_ang_vel"))
                
                
                # Add variables for tracking detection memory
                last_detection_time = time.time()
                detection_memory_duration = 2.0  # 2 seconds of memory
                detection_active = False

                
                # AIMING/NAVIGATION LOOP
                while (rclpy.ok()) and (self.abort_check is False):
                    # Allow ROS callbacks to process
       
                    twist = Twist()

                    r: tuple[tuple[float, float], tuple[float, float], tuple[float, float], tuple[float, float]] = self.aimer.snapshot_bbox()
                    try:
                        self.aimer.update(r[0], r[1], r[2], r[3])
                    except Exception as e:
                        pass
                    
                    # Check if we have valid values from the aimer
                    if self.aimer.linear_v is not None and self.aimer.angular_v is not None:
                        # We have a detection, update the timer
                        last_detection_time = time.time()
                        detection_active = True
                        
                        # Check if we've reached the target
                        if self.aimer.linear_v == 0 and self.aimer.angular_v == 0:
                            if first_time:
                                first_time=False
                                initial_time=time.time()
                            
                            while abs(initial_time-time.time()) < 0.7:
                                msg = Twist()
                                msg.linear.x=self.lin_vel
                                self.drive_publisher.publish(msg)
                                print("final homing movement",abs(initial_time-time.time()) )
                                time.sleep(0.1)
                            twist.linear.x = float(0.0)
                            twist.angular.z = float(0.0)
                            self.drive_publisher.publish(twist)
                            mission_update = MissionState()
                            mission_update.state = "ARUCO_REACHED" 
                            self.pub.publish(mission_update)
                            return
                            
                        aruco_distance_est = self.aimer.calculate_area(self.aimer.aruco_top_left, self.aimer.aruco_top_right, self.aimer.aruco_bottom_left, self.aimer.aruco_bottom_right)
                        print("here is aruco distance ", aruco_distance_est)
                        if self.aimer.angular_v == 1:
                            twist.angular.z = float(self.aimer.scale_velocity(True, aruco_distance_est))
                            if twist.angular.z < 0.3:
                                twist.angular.z = 0.3
                            print ("firstd if",self.aimer.scale_velocity(True, aruco_distance_est))
                            twist.linear.x = 0.0
                        elif self.aimer.angular_v == -1:
                            twist.angular.z = float(-self.aimer.scale_velocity(True, aruco_distance_est))
                            if twist.angular.z > -0.3:
                                twist.angular.z = -0.3
                            twist.linear.x = 0.0
                        elif self.aimer.linear_v == 1:
                            print ("second check",self.aimer.scale_velocity(False, aruco_distance_est))
                            twist.linear.x = float(self.aimer.scale_velocity(False, aruco_distance_est))
                            if twist.linear.x < 0.2:
                                twist.linear.x = 0.2
                            twist.angular.z = 0.0

                    else:
                        if detection_active and time.time() - last_detection_time < detection_memory_duration:
                            print("Using last movement commands from memory")
                        else:
                            print("Detection lost and memory expired, returning to grid search")
                            detection_active = False
                            break
                    
                    self.drive_publisher.publish(twist)
                    # time.sleep(0.1)
                    time.sleep(0.1)
                break

            self.drive_publisher.publish(msg)
            # time.sleep(0.1)
            time.sleep(0.1)

    def move_to_target_avoidance(self, target_x, target_y, current_state): #navigate needs to take in a state value as well (FINISHIT)
        avoidance_msg = MissionState()
        avoidance_msg.state = "GS_WAYPOINT_AVOIDANCE"
        avoidance_msg.current_goal = PoseStamped()
        avoidance_msg.current_goal.pose.position.x = target_x
        avoidance_msg.current_goal.pose.position.y = target_y
        # avoidance_msg.current_goal = [target_x, target_y]
        self.state = None
        self.pub.publish(avoidance_msg)
        # self.get_logger().info("Grid Search in AVOIDANCE IN PROGRESS3333!!!")
        obj = self.mallet_found or self.waterbottle_found or self.hammer_found
        
        mapping = {"AR1":self.aruco_found, 
                   "AR2":self.aruco_found,
                   "OBJ1":obj,
                   "OBJ2":obj, 
                   "OBJ3":obj}

        
        while (rclpy.ok()) and (self.abort_check is False):
            # Allow ROS callbacks to process during the tight loop
            obj = self.mallet_found or self.waterbottle_found or self.hammer_found
            mapping = {"AR1":self.aruco_found, 
                   "AR2":self.aruco_found,
                   "OBJ1":obj,
                   "OBJ2":obj, 
                   "OBJ3":obj}
            
            if mapping[current_state] is False: # while not detected
                if target_x is None or target_y is None or self.x is None or self.y is None:
                    drive_msg = Twist()
                    drive_msg.linear.x = float(0)
                    drive_msg.angular.z = float(0)
                    self.drive_publisher.publish(drive_msg)
                    
                    print("target_x, target_y ,self.x, self.y is none.")
                    continue
            
            if self.state == "GS_WAYPOINT_REACHED":
                # self.get_logger().info("Grid SEARCH WAYPOINT REACHEDDD in GSS")
                # Ensure rover stops immediately before exiting
                stop_msg = Twist()
                stop_msg.linear.x = float(0.0)
                stop_msg.angular.z = float(0.0)
                self.drive_publisher.publish(stop_msg)
                # clear avoidance flag and exit
                self.avoidance_in_progress = False
                self.state = None
                return
            
            # time.sleep(0.1)
            time.sleep(0.1)


    def turn_to_find(self, heading_delta, current_state):
        kp = 0.5
        angle_threshold = 0.2

        # logic here might not be ideal - could be some weird scenario where the state is not one of these despite 
        # only being called when grid_search is required
        obj = self.mallet_found or self.waterbottle_found or self.hammer_found
        
        mapping = {"AR1":self.aruco_found, 
                   "AR2":self.aruco_found,
                   "OBJ1":obj,
                   "OBJ2":obj, 
                   "OBJ3":obj}
        first_time=True
        init_angle = self.heading
        init_angle2 = self.heading2
        target_heading = (init_angle + heading_delta) % (2 * math.pi)
        target_heading2 = (init_angle2 + heading_delta) % (2 * math.pi)        
        while (rclpy.ok()) and (self.abort_check is False):
            # Allow ROS callbacks to process during the tight loop
            obj = self.mallet_found or self.waterbottle_found or self.hammer_found
            mapping = {"AR1":self.aruco_found, 
                   "AR2":self.aruco_found,
                   "OBJ1":obj,
                   "OBJ2":obj, 
                   "OBJ3":obj}
            msg = Twist()
            if mapping[current_state] is False: # while not detected
                # normal operations
                if heading_delta is None or self.heading is None or self.heading2 is None:
                    print("heading is none.")
                    continue

                if abs(target_heading - self.heading) < abs(target_heading2 - self.heading2):
                    angle_diff = target_heading - self.heading
                else:
                    angle_diff = target_heading2 - self.heading2

                if angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                elif angle_diff < -math.pi:
                    angle_diff += 2 * math.pi

                if abs(angle_diff) < angle_threshold:
                    msg.linear.x = float(0)
                    msg.angular.z = float(0)
                    self.drive_publisher.publish(msg)
                    print(f"Reached target turn angle")
                    break

                
                # print("adjusting angle")
                msg.linear.x = float(0)
                msg.angular.z = float(0.3) * math.copysign(1, angle_diff) #0.4 if angle_diff > 0 else -0.4 
                self.drive_publisher.publish(msg)

            elif self.aruco_found: #if mapping[state] is True and is an aruco tag is found 
                print("IN HOMING! mapping state is true!")
                message="In Homing"
                # self.message_pub.publish(message)
                self.message_pub.publish(StdString(data=message))

                if self.aruco_bbox_sub is None:
                        # self.aruco_bbox_sub= self.create_subscription(Float64MultiArray, 'aruco_node/bbox', aimer.rosUpdate, 10) 
                    print (sm_config.get("Ar_homing_lin_vel"),sm_config.get("Ar_homing_ang_vel"))
                
                # # Wait a bit for initial detection
                # for i in range(50):

                #     time.sleep(0.1)
                
                # Add variables for tracking detection memory
                last_detection_time = time.time()
                detection_memory_duration = 2.0  # 2 seconds of memory
                detection_active = False

                while (rclpy.ok()) and (self.abort_check is False):
                    # Allow ROS callbacks to process
       
                    twist = Twist()

                    r: tuple[tuple[float, float], tuple[float, float], tuple[float, float], tuple[float, float]] = self.aimer.snapshot_bbox()
                    try:
                        self.aimer.update(r[0], r[1], r[2], r[3])
                    except Exception as e:
                        pass
                    
                    # Check if we have valid values from the aimer
                    if self.aimer.linear_v is not None and self.aimer.angular_v is not None:
                        # We have a detection, update the timer
                        last_detection_time = time.time()
                        detection_active = True
                        
                        # Check if we've reached the target
                        if self.aimer.linear_v == 0 and self.aimer.angular_v == 0:
                            if first_time:
                                first_time=False
                                initial_time=time.time()
                            
                            while abs(initial_time-time.time()) < 0.7:
                                msg = Twist()
                                msg.linear.x=self.lin_vel
                                self.drive_publisher.publish(msg)
                                print("final homing movement",abs(initial_time-time.time()) )
                                time.sleep(0.1)
                            twist.linear.x = float(0.0)
                            twist.angular.z = float(0.0)
                            self.drive_publisher.publish(twist)
                            mission_update = MissionState()
                            mission_update.state = "ARUCO_REACHED" 
                            self.pub.publish(mission_update)
                            return
                            
                        aruco_distance_est = self.aimer.calculate_area(self.aimer.aruco_top_left, self.aimer.aruco_top_right, self.aimer.aruco_bottom_left, self.aimer.aruco_bottom_right)
                        print("here is aruco distance ", aruco_distance_est)
                        if self.aimer.angular_v == 1:
                            twist.angular.z = float(self.aimer.scale_velocity(True, aruco_distance_est))
                            if twist.angular.z < 0.2:
                                twist.angular.z = 0.2
                            print ("firstd if",self.aimer.scale_velocity(True, aruco_distance_est))
                            twist.linear.x = 0.0
                        elif self.aimer.angular_v == -1:
                            twist.angular.z = float(-self.aimer.scale_velocity(True, aruco_distance_est))
                            if twist.angular.z > -0.2:
                                twist.angular.z = -0.2
                            twist.linear.x = 0.0
                        elif self.aimer.linear_v == 1:
                            print ("second check",self.aimer.scale_velocity(False, aruco_distance_est))
                            twist.linear.x = float(self.aimer.scale_velocity(False, aruco_distance_est))
                            if twist.linear.x < 0.2:
                                twist.linear.x = 0.2
                            twist.angular.z = 0.0
                    else:
                        if detection_active and time.time() - last_detection_time < detection_memory_duration:
                            print("Using last movement commands from memory")
                        else:
                            print("Detection lost and memory expired, returning to grid search")
                            detection_active = False
                            break
                    
                    self.drive_publisher.publish(twist)
                    # time.sleep(0.1)
                    time.sleep(0.1)
                break

            self.drive_publisher.publish(msg)
            # time.sleep(0.1)
            time.sleep(0.1)


    def navigate(self): #navigate needs to take in a state value as well, default value is Location Selection
        self.aruco_found = False
        self.mallet_found = False
        self.waterbottle_found = False
        self.hammer_found = False
        self.count = 0
        self.timer = 0 
        
        self.found_objects = {"AR1":False, 
                   "AR2":False,
                   "OBJ1":False,
                   "OBJ2":False, 
                   "OBJ3":False}
        self.targets = self.square_target()
        while self.current_state not in [ "AR1", "AR2", "OBJ1", "OBJ2", "OBJ3" ]: 
            self.get_logger().info("Waiting for valid current_state before starting navigation...")
            time.sleep(0.1)
        print("targets generated:", self.targets)
        # Guard: ensure we have a valid current_state before navigating
        if not self.current_state or self.current_state not in self.found_objects:
            self.get_logger().warn(f"navigate called with invalid current_state: '{self.current_state}'")
            return False
        msg = MissionState()
        heading = math.pi*2/3
        # if self.current_state == "OBJ1" or self.current_state == "OBJ2":
        #     for h in range(3):
        #         if self.found_objects.get(self.current_state, False): #should be one of aruco, mallet, waterbottle
        #             print(f"Object detected during navigation: {self.found_objects.get(self.current_state)}")
        #             msg.state = "OBJ_FOUND"
        #             self.pub.publish(msg)
        #             return True
        #         self.turn_to_find(heading, self.current_state)
        #         if self.abort_check:
        #             print("self.abort is true!")
        #             break
        for target_x, target_y in self.targets:
            # print('self target lenght', len(self.targets), self.targets, target_x,target_y)
            if self.found_objects.get(self.current_state, False): #should be one of aruco, mallet, waterbottle
                print(f"Object detected during navigation: {self.found_objects.get(self.current_state)}")
                msg.state = "OBJ_FOUND"
                self.pub.publish(msg)
                return True
            # print("Going to target", target_x, target_y)
            
            """TESTING: returning a success status for grid search. 
            success = self.move_to_target(target_x, target_y, self.current_state)
            if not success:
               self.get_logger().warn(f"Failed to reach target ({target_x}, {target_y}). Skipping to next.")
               continue # Skip to the next waypoint in the grid search
            """
            if self.avoidance_in_progress:
                # self.get_logger().info(f"WOOOODDDBINE {target_x}, {target_y}...")
                self.move_to_target_avoidance(target_x, target_y, self.current_state)
               
            else:
                self.move_to_target(target_x, target_y, self.current_state) #changed from navigate_to_target

            if self.abort_check:
                print("self.abort is true!")
                break
            # time.sleep(0.1)
            time.sleep(1)  # Small delay between targets

        
        msg = MissionState()
        if self.found_objects.get(self.current_state, False):
            msg.state="OBJ_FOUND"
            self.pub.publish(msg)
            return True
        msg.state="OBJ_NOT_FOUND"
        self.pub.publish(msg)
        return False 
    
def main():
    import rclpy
    rclpy.init()
    executor = MultiThreadedExecutor()
    gs = GS_Traversal()
    executor.add_node(gs)
    try:
        executor.spin()
    finally:
        gs.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()