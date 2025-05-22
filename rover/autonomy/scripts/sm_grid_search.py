#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Bool
import math
from nav_msgs.msg import Odometry
import aruco_homing as aruco_homing
import ar_detection_node as ar_detect
import sm_straight_line as StraightLineApproach

import yaml
import os
import time

file_path = os.path.join(os.path.dirname(__file__), "sm_config.yaml")

with open(file_path, "r") as f:
    sm_config = yaml.safe_load(f)


# File for the grid search class. Class is initialized with linear, angular velocities, and the target location sent through
# the state machine. When the function square_target is called, it generates grid search targets in a spiral to navigate to. When 
# navigate function is called it calls the move_to_target function on the generated square targets, tries to navigate to each of them. 
# When aruco, waterbottle, or mallet callback functions detect an object in the corresponding state, it calls aruco homing
# to navigate to the detected item to get close enough
    
class GS_Traversal:
    def __init__(self, lin_vel, ang_vel, targets, state):
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel
        self.targets = targets
        self.found = False 
        self.abort_check = False
        self.x = 0
        self.timer=0
        self.y = 0
        self.heading = 0
        self.state = state
        self.count = 0
        
        self.aruco_found = False
        self.mallet_found = False
        self.waterbottle_found = False

        # modified code: add dictionary to manage detection flags for multiple objects
        self.found_objects = {"AR1":False, 
                   "AR2":False,
                   "AR3":False,
                   "OBJ1":False,
                   "OBJ2":False}
        
        # self.odom_subscriber = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        self.pose_subscriber = rospy.Subscriber('pose', PoseStamped, self.pose_callback)
        self.target_subscriber = rospy.Subscriber('target', Float64MultiArray, self.target_callback)
        self.drive_publisher = rospy.Publisher('/drive', Twist, queue_size=10)

        #new additions
        self.aruco_sub = rospy.Subscriber("aruco_found", Bool, callback=self.aruco_detection_callback)
        self.mallet_sub = rospy.Subscriber('mallet_detected', Bool, callback=self.mallet_detection_callback)
        self.waterbottle_sub = rospy.Subscriber('waterbottle_detected', Bool, callback=self.waterbottle_detection_callback)
        self.abort_sub = rospy.Subscriber("auto_abort_check", Bool, self.abort_callback)

        # self.object_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        
    def pose_callback(self, msg):
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

    def move_to_target(self, target_x, target_y, state): #navigate needs to take in a state value as well (FINISHIT)
        rate = rospy.Rate(50)
        kp = 0.5
        threshold = 1
        angle_threshold = 0.2

        # logic here might not be ideal - could be some weird scenario where the state is not one of these despite 
        # only being called when grid_search is required
        obj = self.mallet_found or self.waterbottle_found
        
        mapping = {"AR1":self.aruco_found, 
                   "AR2":self.aruco_found,
                   "AR3":self.aruco_found,
                   "OBJ1":obj,
                   "OBJ2":obj}
        homing_done=False
        first_time=True
        
        # print("In move to target")
        while (not rospy.is_shutdown()) and (self.abort_check is False):
            obj = self.mallet_found or self.waterbottle_found
            mapping = {"AR1":self.aruco_found, 
                   "AR2":self.aruco_found,
                   "AR3":self.aruco_found,
                   "OBJ1":obj,
                   "OBJ2":obj}
            # print("in grid search: mapping state", mapping[state])
            msg = Twist()
            # print("state", state)
            # print("mapping state", mapping[state])
            pub = rospy.Publisher('drive', Twist, queue_size=10)
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
                        
                # print("angular velocity", msg.angular.z)

            else: #if mapping[state] is True --> if the object is found
                print("mapping state is true!")
                print("IN HOMING")
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
           
                
            # print("publishing grid search velocity")
            self.drive_publisher.publish(msg)
            rate.sleep()

    def navigate(self): #navigate needs to take in a state value as well, default value is Location Selection
        
        for target_x, target_y in self.targets:
            print('self target lenght', len(self.targets), self.targets, target_x,target_y)
            if self.found_objects[self.state]: #should be one of aruco, mallet, waterbottle
                print(f"Object detected during navigation: {self.found_objects[self.state]}")
                return True
            
            print("Going to target", target_x, target_y)
            self.move_to_target(target_x, target_y, self.state) #changed from navigate_to_target
            if self.abort_check:
                print("self.abort is true!")
                break

            rospy.sleep(1)

        if self.found_objects[self.state]:
            
            return True
        return False #Signalling that grid search has been done

class GridSearch:
    '''
    All values here should be treated as doubles
    w, h - refers to grid dimensions, ie. if comp tells us that it is within radius of 20 then our grid would be square with 40 by 40
    tolerance   - this is the limitation of our own equipment, ie. from camera, aruco only detected left right and dist of 3 m
                - if this is unequal, take the smallest tolerance
    '''
    def __init__(self, w, h, tolerance, start_x, start_y): # AR1, OR AR2 (cartesian - fixed)
        self.x = w 
        self.y = h
        self.tol = tolerance 
        # print(self.tol)
        self.start_x = start_x
        self.start_y = start_y
        # rospy.Subscriber("/rtabmap/odom", Odometry, self.odom_callback) # change topic name

    '''
    Generates a list of targets for square straight line approach.
    '''
    def square_target(self):
        # dx, dy indicates direction (goes "up" first)
        dx, dy = 0, 1
        # while self.start_x == None:
        #     # print("Waiting for odom...")
        #     # rospy.loginfo("Waiting for odom...")
        #     continue
        targets = [(self.start_x,self.start_y)]
        step = 1
        out_of_bounds = False

        while len(targets) < self.x**2 - 1:
            for i in range(2):  # 1, 1, 2, 2, 3, 3... etc
                for j in range(step):
                    if step*self.tol > self.x: 
                        print ("step is outside of accepted width: step*tol=", step*self.tol, "\tself.x=",self.x)
                        out_of_bounds = True
                        break
                    self.start_x, self.start_y = self.start_x + (self.tol*dx), self.start_y + (self.tol*dy)
                    targets.append((self.start_x, self.start_y))
                dx, dy = -dy, dx
                if out_of_bounds:
                    break
            step += 1 
            if out_of_bounds:
                break
        print ("these are the final targets", targets) 
        return targets

    #Which function should be used?
    def alt_gs(self):
        dx, dy = 0, 1
        # while self.start_x == None:
        #     # print("Waiting for odom...")
        #     # rospy.loginfo("Waiting for odom...")
        #     continue
        targets = [(self.start_x,self.start_y)]
        step = 1
        # notice, up/down is odd amount, left/right is even amount        
        while len(targets) < self.x: # 1, 2, 3, 4, 5... sol 
            if step%2==1: # moving up/down
                for i in range (step):
                    # move up if %4==1, down if %4 ==3 from the pattern
                    self.start_y+=1 if step%4 ==1 else -1
                    targets.append((self.start_x,self.start_y))
                    if len(targets)==self.x:
                        return targets
            else: 
                for i in range (step):
                # move right if %4==2, left if %4 ==0 
                    self.start_x+=1 if step%4==2 else -1
                    targets.append((self.start_x,self.start_y))
                    if len(targets)==self.x:
                        return targets
            step += 1  # Increase step size after two edges
        return targets

def main():
    targets = [(2, 0)]  # Define multiple target points
    try:
        rospy.init_node('straight_line_approach_node')
        approach = StraightLineApproach.StraightLineApproach(1.5, 0.5, targets)
        approach.navigate()
    except rospy.ROSInterruptException:
        pass

    gs = GridSearch(10, 10, 1, 0, 0)  # define multiple target points here: cartesian
    targets = gs.square_target() #generates multiple targets 
    gs_traversal_object = GS_Traversal(0.6, 0.3, targets, "AR1")
    gs_traversal_object.navigate() #should be one of aruco, mallet, waterbottle
    
    pub = rospy.Publisher('drive', Twist, queue_size=10) # change topic name
    # frame_width, frame_height, min_aruco_area, aruco_min_x_uncert, aruco_min_area_uncert, max_linear_v, max_angular_v
    aimer = aruco_homing.AimerROS(640, 360, 1000, 100, 100, 1.5, 0.3) # FOR ARUCO
    
    # aimer = aruco_homing.AimerROS(50, 50, 1450, 10, 50, 1.0, 0.5) # FOR WATER BOTTLE
    rospy.Subscriber('aruco_node/bbox', Float64MultiArray, callback=aimer.rosUpdate) # change topic name
    # int32multiarray convention: [top_left_x, top_left_y, top_right_x, top_right_y, bottom_left_x, bottom_left_y, bottom_right_x, bottom_right_y]
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        twist = Twist()
        if aimer.linear_v == 0 and aimer.angular_v == 0:
            print ("at weird", aimer.linear_v, aimer.angular_v)
            twist.linear.x = 0
            twist.angular.z = 0
            pub.publish(twist)
            return True
        if aimer.angular_v == 1:
            twist.angular.z = aimer.max_angular_v
            twist.linear.x = 0
        elif aimer.angular_v == -1:
            twist.angular.z = -aimer.max_angular_v
            twist.linear.x = 0
        elif aimer.linear_v == 1:
            twist.linear.x = aimer.max_linear_v
            twist.angular.z = 0
         
        pub.publish(twist)
        rate.sleep()
    
    
if __name__ == '__main__':
    main()