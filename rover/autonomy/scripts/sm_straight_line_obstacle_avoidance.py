#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Bool, Float32MultiArray
from astar_obstacle_avoidance_algorithim import AstarObstacleAvoidance
import math
import ar_detection_node as adn

import yaml
import os

file_path = os.path.join(os.path.dirname(__file__), "sm_config.yaml")

with open(file_path, "r") as f:
    sm_config = yaml.safe_load(f)
    
class StraightLineObstacleAvoidance:
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
        self.aruco_sub = rospy.Subscriber("aruco_found", Bool, callback=self.detection_callback)
        self.astar_sub = rospy.Subscriber('/astar_waypoints', Float32MultiArray, self.astar_callback)
        self.waypoints = []

    def pose_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.heading = self.to_euler_angles(msg.pose.orientation.w, msg.pose.orientation.x, 
                                            msg.pose.orientation.y, msg.pose.orientation.z)[2]

    def abort_callback(self,msg):
        self.abort_check = msg.data
        
    def astar_callback(self, msg):
        data = msg.data
        self.waypoints = [(data[i], data[i+1]) for i in range(0, len(data), 2)]
        print("recieved waypoints", self.waypoints)
        rospy.loginfo(f"New path received with {len(self.waypoints)} waypoints")

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

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def move_to_target(self, target_x, target_y, state="Location Selection"):
        rate = rospy.Rate(50)
        threshold = 0.5  # meters
        angle_threshold = 0.2  # radians
        kp = 0.5  # Angular proportional gain

        while not rospy.is_shutdown() and not self.abort_check:
            if not self.waypoints or self.x is None:
                rate.sleep()
                continue

            current_target = self.waypoints[0]
            target_x, target_y = current_target

            # Calculate target direction and distance
            target_heading = math.atan2(target_y - self.y, target_x - self.x)
            angle_diff = self.normalize_angle(target_heading - self.heading)
            distance = math.hypot(target_x - self.x, target_y - self.y)

            # Create Twist message
            msg = Twist()

            if distance < threshold:
                self.waypoints.pop(0)
                rospy.loginfo("Reached waypoint. Proceeding to next.")
                continue

            if abs(angle_diff) > angle_threshold:
                # Rotate in place
                msg.angular.z = max(min(kp * angle_diff, self.ang_vel), -self.ang_vel)
                # Minimum angular velocity to overcome friction
                if abs(msg.angular.z) < 0.3:
                    msg.angular.z = 0.3 if msg.angular.z > 0 else -0.3
            else:
                # Move forward
                msg.linear.x = self.lin_vel

            self.drive_publisher.publish(msg)
            rate.sleep()

        # Stop when done
        self.drive_publisher.publish(Twist())
        
    def navigate(self, state="Location Selection"):
        planner = AstarObstacleAvoidance(self.targets[0])
        planner.run()
        
        while not rospy.is_shutdown() and not self.abort_check:
            if self.targets:
                print("in navigate")
                # Continuously process the first target in the queue
                target_x, target_y = self.targets[0]
                self.move_to_target(target_x, target_y)
            else:
                rospy.loginfo("Waiting for A* path...")
                rospy.sleep(1)
                
    def navigate_old(self, state="Location Selection"): #navigate needs to take in a state value as well
        for target_x, target_y in self.targets:
            print(f"Moving towards target: ({target_x}, {target_y})")
            self.move_to_target(target_x, target_y)
            if self.abort_check:
                self.abort_check = False
                break
            rospy.sleep(1)
                

if __name__ == '__main__':
    targets = [(9, 2)]  
    try:
        rospy.init_node('straight_line_approach_node')
        approach = StraightLineObstacleAvoidance(0, 0, targets)
        approach.navigate()
    except rospy.ROSInterruptException:
        pass