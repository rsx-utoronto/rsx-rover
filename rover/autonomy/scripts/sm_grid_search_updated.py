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
import aruco_homing as aruco_homing
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
        self.state = None
        self.count = 0
        self.w = sm_config.get("AR_grid_search_w") #new
        self.h = sm_config.get("AR_grid_search_h") #new
        self.tol = sm_config.get("AR_grid_search_tol") #new
        self.start_x=0 #new
        self.start_y=0 #new
        self.aruco_found = False
        self.mallet_found = False
        self.waterbottle_found = False
        self.homing_status = None #new
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
        self.abort_sub = self.create_subscription(Bool, "auto_abort_check", self.abort_callback, 10)
        self.message_pub = self.create_publisher(String, "gui_status", 10)
        if sm_config.get("realsense_detection"): #moved here
            aimer = aruco_homing.AimerROS(640, 360, 2500, 100, 100, sm_config.get("Ar_homing_lin_vel") , sm_config.get("Ar_homing_ang_vel")) # FOR ARUCO
        else: 
            aimer = aruco_homing.AimerROS(640, 360, 700, 100, 100, sm_config.get("Ar_homing_lin_vel") , sm_config.get("Ar_homing_ang_vel")) # FOR ARUCO
        
        self.aruco_bbox_sub= self.create_subscription(Float64MultiArray, 'aruco_node/bbox', aimer.rosUpdate, 10) #moved here
        # self.object_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        self.pub = self.create_publisher(MissionState, 'mission_state', 10) #new for integrating with the state machine
        self.create_subscription(MissionState,'mission_state',self.feedback_callback, 10)
        self._nav_thread = None  #new
        self.aimer_aruco=None #new
        self.aimer_object=None #new
        self.aruco_bbox_sub=None #new
        self.object_bbox_sub=None #new

    def feedback_callback(self, msg):
        print("in GS traversal feedback callback, msg.state:", msg.state)
        if msg.state == "START_GS_TRAV":
            self.active = True
            self.state= msg.current_state
            self.get_logger().info("Grid Search behavior ACTIVE")
            self.start_x = msg.current_goal.pose.position.x
            self.start_y = msg.current_goal.pose.position.y
            if self._nav_thread is None or not self._nav_thread.is_alive():
                self._nav_thread = threading.Thread(target=self.navigate, daemon=True)
                self._nav_thread.start()
        # elif msg.state in ("HOMING_DONE", "HOMING_SUCCESS", "HOMING_FAILED"):
        #     self.homing_status = msg.state
        else:
            self.active = False
       
    def 
    (self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.heading = self.to_euler_angles(msg.pose.orientation.w, msg.pose.orientation.x, 
                                            msg.pose.orientation.y, msg.pose.orientation.z)[2]
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.heading = self.to_euler_angles(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)[2]
    
    def abort_callback(self,msg):
        self.abort_check = msg.data

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
            time_now=time.time()
            if abs(self.timer-time_now) >5:
                self.timer=time_now
                self.count=0
            print("count,",self.count)
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

    def square_target(self):
        print("Generating square grid search targets...")
        dx, dy = 0, 1
        # use local copies so we don't mutate the node's starting point
        sx, sy = float(self.start_x), float(self.start_y)
        targets = [(sx, sy)]
        step = 1
        out_of_bounds = False

        while len(targets) < int(self.w)**2 - 1:
            for _ in range(2):
                for _ in range(step):
                    if step * float(self.tol) > float(self.w):
                        out_of_bounds = True
                        break
                    sx, sy = sx + (self.tol * dx), sy + (self.tol * dy)
                    targets.append((sx, sy))
                dx, dy = -dy, dx
                if out_of_bounds:
                    break
            step += 1
            if out_of_bounds:
                break

        print("these are the final targets", targets)
        return targets
    
    def move_to_target(self, target_x, target_y, state): #navigate needs to take in a state value as well (FINISHIT)
        kp = 0.5
        threshold = 1
        angle_threshold = 0.2

        # logic here might not be ideal - could be some weird scenario where the state is not one of these despite 
        # only being called when grid_search is required
        obj = self.mallet_found or self.waterbottle_found
        
        mapping = {"AR1":self.aruco_found, 
                   "AR2":self.aruco_found,
                   "OBJ1":obj,
                   "OBJ2":obj, 
                   "OBJ3":obj}
        first_time=True
        
        while (rclpy.ok()) and (self.abort_check is False):
            # Allow ROS callbacks to process during the tight loop
         
            
            obj = self.mallet_found or self.waterbottle_found
            mapping = {"AR1":self.aruco_found, 
                   "AR2":self.aruco_found,
                   "OBJ1":obj,
                   "OBJ2":obj, 
                   "OBJ3":obj}
            msg = Twist()
            if mapping[state] is False: #while not detected
                # normal operations
                if target_x is None or target_y is None or self.x is None or self.y is None:
                    print("tagret x, taget y ,self.x, self.y is none ")
                    continue

                target_heading = math.atan2(target_y - self.y, target_x - self.x)
                target_distance = math.sqrt((target_x - self.x) ** 2 + (target_y - self.y) ** 2)
                print("Target Heading:", math.degrees(target_heading), " Target Distance:", target_distance)
                angle_diff = target_heading - self.heading

                if angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                elif angle_diff < -math.pi:
                    angle_diff += 2 * math.pi

                if target_distance < threshold:
                    msg.linear.x = float(0)
                    msg.angular.z = float(0)
                    self.drive_publisher.publish(msg)
                    print(f"Reached target: ({target_x}, {target_y})")
                    break 

                if abs(angle_diff) <= angle_threshold:
                    msg.linear.x = self.lin_vel
                    msg.angular.z = float(0)
                else:
                    msg.linear.x = float(0)
                    msg.angular.z = angle_diff * kp
                    if abs(msg.angular.z) < 0.3:
                        msg.angular.z = 0 #0.3 if msg.angular.z > 0 else -0.3 CHANGE WHEN TESTING OUSTIDE

            else: #if mapping[state] is True --> if the object is found
                print("mapping state is true!")
                print("IN HOMING")
                message="In Homing"
                # self.message_pub.publish(message)
                self.message_pub.publish(StdString(data=message))
                # call homing
                # should publish that it is found
                if state == "AR1" or state == "AR2":
                    # this sees which camera it is using and then uses the parameters accordingly.
                    if self.aruco_bbox_sub is None:
                        # self.aruco_bbox_sub= self.create_subscription(Float64MultiArray, 'aruco_node/bbox', aimer.rosUpdate, 10) 
                        print (sm_config.get("Ar_homing_lin_vel"),sm_config.get("Ar_homing_ang_vel"))
                    
                elif state == "OBJ1" or state == "OBJ2" or state == "OBJ3" :
                    aimer = aruco_homing.AimerROS(640, 360, 1450, 100, 200, sm_config.get("Obj_homing_lin_vel"), sm_config.get("Obj_homing_ang_vel")) # FOR WATER BOTTLE, MALLET
                    # if self.object_bbox_sub is None:
                        # self.object_bbox_sub= self.create_subscription(Float64MultiArray, 'object/bbox', aimer.rosUpdate, 10)
                    print (sm_config.get("Obj_homing_lin_vel"),sm_config.get("Obj_homing_ang_vel"))
              
                # Wait a bit for initial detection
                for i in range(50):

                    time.sleep(0.1)
                
                # Add variables for tracking detection memory
                last_detection_time = time.time()
                detection_memory_duration = 2.0  # 2 seconds of memory
                detection_active = False

                while (rclpy.ok()) and (self.abort_check is False):
                    # Allow ROS callbacks to process
       
                    
                    twist = Twist()
                    
                    # Check if we have valid values from the aimer
                    if aimer.linear_v is not None and aimer.angular_v is not None:
                        # We have a detection, update the timer
                        last_detection_time = time.time()
                        detection_active = True
                        
                        # Check if we've reached the target
                        if aimer.linear_v == 0 and aimer.angular_v == 0:
                            if first_time:
                                first_time=False
                                initial_time=time.time()
                            
                            while abs(initial_time-time.time()) < 0.7:
                            
                                msg.linear.x=self.lin_vel
                                self.drive_publisher.publish(msg)
                                print("final homing movement",abs(initial_time-time.time()) )
                                time.sleep(0.1)
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                            self.drive_publisher.publish(twist)
                            return
                            
                        # Normal homing behavior
                        if aimer.angular_v == 1:
                            twist.angular.z = float(aimer.max_angular_v)
                            twist.linear.x = 0.0
                        elif aimer.angular_v == -1:
                            twist.angular.z = float(-aimer.max_angular_v)
                            twist.linear.x = 0.0
                        elif aimer.linear_v == 1:
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
                    # time.sleep(0.1)
                    time.sleep(0.1)
                break

            self.drive_publisher.publish(msg)
            # time.sleep(0.1)
            time.sleep(0.1)

    def navigate(self): #navigate needs to take in a state value as well, default value is Location Selection
        print("in gs navigate")
        msg = MissionState()
        self.targets = self.square_target()
        print("targets generated:", self.targets)
        
        twist = Twist() #code for spinning in a circle initially
        initz_heading = self.heading[2]
        while ((self.heading-initz_heading) > 5.9): #If the angle difference is 20degrees
            twist.angular.z = self.ang_vel
            self.drive_publisher(twist)
            if self.found_objects[self.state]: #should be one of aruco, mallet, waterbottle
                if self.state == "OBJ1" or self.state == "OBJ2" or self.state == "OBJ3": #if objects detected are the an Object
                    print(f"Object detected during navigation: {self.found_objects[self.state]}")
                    msg.state="OBJ_FOUND"
                    self.pub.publish(msg)
                    return True
                else:   #if objects detected are an aruco, should be tested 
                    self.move_to_target(self.x, self.y) #Will aruco found return false? self.x and self.y won't be used
                    if self.found_objects[self.state]: #should be aruco
                        print(f"Object detected during navigation: {self.found_objects[self.state]}")
                        msg.state="OBJ_FOUND"
                        self.pub.publish(msg)
                        return True 
        
        for target_x, target_y in self.targets:
            # print('self target lenght', len(self.targets), self.targets, target_x,target_y)
            if self.found_objects[self.state]: #should be one of aruco, mallet, waterbottle
                print(f"Object detected during navigation: {self.found_objects[self.state]}")
                msg.state="OBJ_FOUND"
                self.pub.publish(msg)
                return True
            # print("Going to target", target_x, target_y)
            self.move_to_target(target_x, target_y, self.state) #changed from navigate_to_target
            if self.abort_check:
                print("self.abort is true!")
                break
            # time.sleep(0.1)
            time.sleep(1)  # Small delay between targets

        
        msg = MissionState()
        if self.found_objects[self.state] and self.abort_check is False:
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
    executor.add_node(gs)
    gs= GS_Traversal() 
    try:
        executor.spin(gs)
    finally:
        gs.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()