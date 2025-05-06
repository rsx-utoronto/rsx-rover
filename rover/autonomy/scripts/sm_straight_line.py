#!/usr/bin/python3

import rospy
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
    
class StraightLineApproach:
    def __init__(self, lin_vel, ang_vel, targets):
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel
        self.targets = targets
        self.found = False
        self.abort_check = False
        self.x = 0
        self.y = 0
        self.heading = 0
        self.pose_subscriber = rospy.Subscriber(sm_config.get("pose_param_straight_line"), PoseStamped, self.pose_callback)
        self.target_subscriber = rospy.Subscriber('target', Float64MultiArray, self.target_callback)
        self.drive_publisher = rospy.Publisher(sm_config.get("drive_param_straight_line"), Twist, queue_size=10)
        self.aruco_found = False
        self.abort_sub = rospy.Subscriber("auto_abort_check", Bool, self.abort_callback)
        #new additions
        # self.aruco_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        self.aruco_sub = rospy.Subscriber("aruco_found", Bool, callback=self.detection_callback)
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
    
    def detection_callback(self, data):
        self.found = data

    def move_to_target(self, target_x, target_y, state="Location Selection"): #navigate needs to take in a state value as well (FINISHIT)
        rate = rospy.Rate(50)
        kp = 0.5
        threshold = 0.5
        angle_threshold = 0.2

        while (not rospy.is_shutdown()) and (self.abort_check is False):
            msg = Twist()
            if target_x is None or target_y is None or self.x is None or self.y is None:
                continue

            target_heading = math.atan2(target_y - self.y, target_x - self.x)
            target_distance = math.sqrt((target_x - self.x) ** 2 + (target_y - self.y) ** 2)
           # print("Target Heading:", math.degrees(target_heading), "Cur Heading:", math.degrees(self.heading))
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
            rate.sleep()

    def navigate(self, state="Location Selection"): #navigate needs to take in a state value as well
        for target_x, target_y in self.targets:
            print(f"Moving towards target: ({target_x}, {target_y})")
            self.move_to_target(target_x, target_y)
            if self.abort_check:
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