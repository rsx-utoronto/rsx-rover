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


class GS_Traversal:
    def __init__(self, lin_vel, ang_vel, targets):
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel
        self.targets = targets
        self.found = False #Do we need?
        self.x = 0
        self.y = 0
        self.heading = 0

        # modified code: add dictionary to manage detection flags for multiple objects
        self.found_objects = {
            "aruco_detected": False,
            "mallet_detected": False,
            "waterbottle_detected": False,
        }
        
        # self.odom_subscriber = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        self.pose_subscriber = rospy.Subscriber('/pose', PoseStamped, self.pose_callback)
        self.target_subscriber = rospy.Subscriber('target', Float64MultiArray, self.target_callback)
        self.drive_publisher = rospy.Publisher('drive', Twist, queue_size=10)

        #new additions
        self.aruco_found = rospy.Subscriber("aruco_found", Bool, callback=self.aruco_detection_callback)
        self.mallet_found = rospy.Subscriber('mallet_detected', Bool, callback=self.mallet_detection_callback)
        self.waterbottle_found = rospy.Subscriber('waterbottle_detected', callback=self.waterbottle_detection_callback)

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
        self.aruco_found = data.data
        self.found_objects["aruco_detected"] = True
    
    def mallet_detection_callback(self, data):
        self.mallet_found = data.data
        self.found_objects["mallet_detected"] = True

    def waterbottle_detection_callback(self, data):
        self.waterbottle_found = data.data
        self.found_objects["waterbottle_detected"] = True

    def move_to_target(self, target_x, target_y, state): #navigate needs to take in a state value as well (FINISHIT)
        rate = rospy.Rate(50)
        kp = 0.5
        threshold = 0.1

        # logic here might not be ideal - could be some weird scenario where the state is not one of these despite 
        # only being called when grid_search is required
        obj = self.mallet_found or self.waterbottle_found
        
        mapping = {"AR1":self.aruco_found, 
                   "AR2":self.aruco_found,
                   "AR3":self.aruco_found,
                   "OBJ1":obj,
                   "OBJ2":obj}

        while not rospy.is_shutdown():
            msg = Twist()
            if mapping[state] is False: #while not detected
                # normal operations
                if target_x is None or target_y is None or self.x is None or self.y is None:
                    continue

                target_heading = math.atan2(target_y - self.y, target_x - self.x)
                target_distance = math.sqrt((target_x - self.x) ** 2 + (target_y - self.y) ** 2)
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

                if abs(angle_diff) <= threshold:
                    msg.linear.x = self.lin_vel
                    msg.angular.z = 0
                else:
                    msg.linear.x = 0
                    msg.angular.z = angle_diff * kp
                    if abs(msg.angular.z) < 0.3:
                        msg.angular.z = 0.3 if msg.angular.z > 0 else -0.3

            else: #if mapping[state] is True --> if the object is found
                # call homing
                # should publish that it is found
                # rospy.init_node('aruco_homing', anonymous=True) # change node name if needed
                pub = rospy.Publisher('drive', Twist, queue_size=10) # change topic name
                aimer = aruco_homing.AimerROS(640, 360, 1000, 100, 100, 0.5, 0.5) # change constants
                rospy.Subscriber('aruco_node/bbox', Float64MultiArray, callback=aimer.rosUpdate) # change topic name
                rate = rospy.Rate(10) #this code needs to be adjusted
                while not rospy.is_shutdown():
                    twist = Twist()
                    if aimer.linear_v == 0 and aimer.angular_v == 0:
                        print ("at weird", aimer.linear_v, aimer.angular_v)
                        twist.linear.x = 0
                        twist.angular.z = 0
                        pub.publish(twist)
                        return
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

                break

            self.drive_publisher.publish(msg)
            rate.sleep()

    def navigate(self, state="Location Selection"): #navigate needs to take in a state value as well, default value is Location Selection
        for target_x, target_y in self.targets:
            if self.found_objects[state]: #should be one of aruco, mallet, waterbottle
                print(f"Object detected during navigation: {state}")
                return True
            self.move_to_target(target_x, target_y) #changed from navigate_to_target

            rospy.sleep(1)

        if self.found_objects[state]:
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
        

    # def odom_callback(self, msg):
    #     self.start_x = msg.pose.pose.position.x
    #     self.start_y = msg.pose.pose.position.y
    #     print(self.start_x)
    #     print(self.start_y)
    
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

        while len(targets) < self.x:
            for i in range(2):  # 1, 1, 2, 2, 3, 3... etc
                for j in range(step):
                    if step*self.tol > self.x: 
                        print ("step is outside of accepted width: step*tol=", step*self.tol, "\tself.x=",self.x)
                        break
                    self.start_x, self.start_y = self.start_x + (self.tol*dx), self.start_y + (self.tol*dy)
                    targets.append((self.start_x, self.start_y))
                dx, dy = -dy, dx
            step += 1 
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
    targets = [(5, -0.8)]  # Define multiple target points
    try:
        rospy.init_node('straight_line_approach_node')
        approach = StraightLineApproach.StraightLineApproach(1.5, 0.5, targets)
        approach.navigate()
    except rospy.ROSInterruptException:
        pass

    # gs = GridSearch(4, 4, 1, 0, 0)  # define multiple target points here: cartesian
    # targets = gs.square_target() #generates multiple targets 
    # gs_traversal_object = GS_Traversal(0.6, 0.3, targets)
    # gs_traversal_object.navigate("AR1") #should be one of aruco, mallet, waterbottle
    
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