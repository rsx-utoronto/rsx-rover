#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Bool
from rover.msg import MissionState
import math
import time
import ar_detection_node as adn
import threading
import yaml
import os

file_path = os.path.join(os.path.dirname(__file__), "sm_config.yaml")
#file_path = "/home/rsx/rover_ws/src/rsx-rover/rover/autonomy/scripts/sm_config.yaml"

with open(file_path, "r") as f:
    sm_config = yaml.safe_load(f)

# File for the straight line traversal class for the state machine. The class is initialized with linear, angular velocities, and target passed
# through the state machine. When navigate function is called, it calls the move_to_target function which continuously calculates the target distance
# and heading to determine the angular and linear velocities. When the target distance is within the threshold it breaks out of the loop to return.
    
class StraightLineApproach(Node):
    def __init__(self):
        super().__init__('straight_line_approach_node')
        # self.lin_vel = lin_vel
        # self.ang_vel = ang_vel
        # self.targets = targets
        self.found = False
        self.abort_check = False
        self.x = -100000
        self.y = -100000
        self.heading = 0
        # self.pose_subscriber = rospy.Subscriber('/pose', PoseStamped, self.pose_callback)
        # self.target_subscriber = rospy.Subscriber('target', Float64MultiArray, self.target_callback)
        # self.drive_publisher = rospy.Publisher('/drive', Twist, queue_size=10)
        self.pose_subscriber = self.create_subscription(PoseStamped, '/pose', self.pose_callback, 1)
        self.target_subscriber = self.create_subscription(Float64MultiArray, '/target', self.target_callback, 1)
        self.drive_publisher = self.create_publisher(Twist, '/drive', 10)
        self.abort_sub = self.create_subscription(Bool, "/auto_abort_check", self.abort_callback, 1)
        self.aruco_found = False
        self.aruco_sub = self.create_subscription(Bool, "/aruco_found", self.detection_callback, 1)
        #new additions
        # self.aruco_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
     
        # self.odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)

        #new additions
        # self.aruco_sub = rospy.Subscriber("aruco_found", Bool, callback=self.aruco_detection_callback)
        # self.mallet_sub = rospy.Subscriber('mallet_detected', Bool, callback=self.mallet_detection_callback)
        # self.waterbottle_sub = rospy.Subscriber('waterbottle_detected', Bool, callback=self.waterbottle_detection_callback)
        self.pub = self.create_publisher(MissionState, 'mission_state', 10)
        self.lin_vel= sm_config.get("straight_line_approach_lin_vel")
        self.ang_vel = sm_config.get("straight_line_approach_ang_vel")
        self.active = False
        self.target = None
        self.create_subscription(MissionState,'mission_state',self.feedback_callback, 10)
        
    def feedback_callback(self, msg):
        if msg.state == "START_SL":
            self.active = True
            self.target = msg.current_goal
            self.get_logger().info("Straight line behavior ACTIVE")
            self.navigate()
        else:
            self.active = False
    
    def pose_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.heading = self.to_euler_angles(msg.pose.orientation.w, msg.pose.orientation.x, 
                                            msg.pose.orientation.y, msg.pose.orientation.z)[2]
        self.get_logger().info(f"x: {self.x}, y {self.y}, heading {self.heading}")
    
    def abort_callback(self,msg):
        self.abort_check = msg.data
        
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.heading = self.to_euler_angles(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)[2]
        self.get_logger().info(f"x: {self.x}, y {self.y}, heading {self.heading}")

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
    
    def detection_callback(self, data):
        self.found = data

    def move_to_target(self, target_x, target_y, state="Location Selection"): #navigate needs to take in a state value as well (FINISHIT)
        
        kp = 0.5
        threshold = 0.5
        angle_threshold = 0.2

        while (rclpy.ok()) and (self.abort_check is False):
            # rclpy.spin_once(self, timeout_sec=0.1)
            msg = Twist()
            if target_x is None or target_y is None or self.x is None or self.y is None:
                continue

            target_heading = math.atan2(target_y - self.y, target_x - self.x)
            target_distance = math.sqrt((target_x - self.x) ** 2 + (target_y - self.y) ** 2)
            # print(f"Current Position: ({self.x}, {self.y})")
            # print("Target Heading:", math.degrees(target_heading), " Target Distance:", target_distance)
            # print(f"x: {self.x}, y {self.y}, heading {self.heading}")

            angle_diff = target_heading - self.heading
            
            # print ( f"angle_diff: {angle_diff}")

            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi
                
            # print (f"diff in heading: {angle_diff}", f"target_distance: {target_distance}")

            if target_distance < threshold:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.drive_publisher.publish(msg)
                print(f"Reached target: ({target_x}, {target_y})")
                break

            if abs(angle_diff) <= angle_threshold:
                msg.linear.x = self.lin_vel
                msg.angular.z = 0.0
            else:
                msg.linear.x = 0.0
                msg.angular.z = angle_diff * kp
                if abs(msg.angular.z) < 0.3:
                    msg.angular.z = 0.3 if msg.angular.z > 0 else -0.3

            self.drive_publisher.publish(msg)
            time.sleep(1/50)

    def navigate(self, state="Location Selection"): #navigate needs to take in a state value as well
        print("self.targets", self.targets)
        for target_x, target_y in self.targets:
            print(f"Moving towards target: ({target_x}, {target_y})")
            self.move_to_target(target_x, target_y)
            if self.abort_check:
                self.abort_check = False
                break
            time.sleep(1)

def main():
    import rclpy
    rclpy.init()
    node = rclpy.create_node('sm_straight_line_idle')
    node.get_logger().info('sm_straight_line alive (idle)')
    try:
        rclpy.spin(node)  # stays active
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()