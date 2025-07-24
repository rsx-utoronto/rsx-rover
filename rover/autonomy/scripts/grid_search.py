#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
# from rover.msg import StateMsg
import math
from transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32
import numpy as np
import time

class grid_search_class(Node):
    def __init__(self):
        super().__init__('aruco_tag_detector')
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.init = 0.0
        self.sub = self.create_subscription(Odometry, "/rtabmap/odom", self.newOdom, 10)
        self.pub = self.create_publisher(Twist, "drive", 10)
        self.pub_error = self.create_publisher(Float32, "/robot_base_velocity_controller/error", 10)
       
        self.speed = Twist()
        # self.roll=euler_from_quaternion([0])
        # self.pitch=euler_from_quaternion([1])
        # self.yaw=euler_from_quaternion([2])
        self.roll, self.pitch, self.yaw = euler_from_quaternion([0, 0, 0, 1])
        self.go_to_loc = False

    def newOdom(self, msg):
        # global x, y, theta, init
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.init += 1

        # print("X: ", x)
        # print("Y: ", y)

        self.rot_q = msg.pose.pose.orientation
       
        #print("ANGLES: ", (roll, pitch, theta))
        # rospy.init_node("speed_controller")
        orientation_list = [self.rot_q.x, self.rot_q.y, self.rot_q.z, self.rot_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list) #new add on 
        self.theta = yaw 

    
    def follow_path(self, path_list, scale_factor, detector):
       

        # path_list = [(0+x,0+y), (3.5+x,0.0+y), (3.5+x, 3.5+y), (-3.5+x, 3.5+y)]
        #             # ,(-3.5+x, -7.0+y), (10.5+x, -7.0+y, (10.5+x,10.5+y), (-10.5+x,10.5+y), 
        #             # (-10.5+x, -14.0+y), (17.5+x, -14.0+y), (17.5+x, 17.5+y), (-17.5+x, 17.5+y), (-17.5+x, -21.0+y), (17.5+x, -21.0+y)]
        point_index = 0  # instead of deleting stuff from a list (which is anyway bug prone) we'll just iterate through it using index variable.
        goal = Point ()
        while rclpy.ok() and not detector.isfound():
            if point_index < len(path_list): # so we won't get an error of trying to reach non-existant index of a list
                goal.x = path_list[point_index][0] + self.x  # x coordinate for goal
                goal.y = path_list[point_index][1] + self.y  # y coordinate for goal
            else:
                self.speed.linear.x = 0.0
                self.speed.angular.z = 0.0
               
                self.nothing_found_at_end()
                 # I guess we're done?
                
            inc_x = (goal.x - self.x)*scale_factor
            inc_y = (goal.y - self.y)*scale_factor 

            if (self.x==0 or self.y==0):
                angle_to_goal = math.pi/2
            else:
                angle_to_goal = math.atan2 (inc_y, inc_x) # this is our "bearing to goal" as I can guess

            # if your position is changing and 0,0 is only the start point then use the distance between 2 points formula
            point_distance_to_goal = np.sqrt(inc_x*inc_x + inc_y*inc_y)

            if point_distance_to_goal >= 0.5: # we'll now head to our target
                if angle_to_goal - self.theta > 0.1:
                    self.speed.linear.x = 0.0
                    self.speed.angular.z = 0.3   
                elif angle_to_goal - self.theta < -0.1:
                    self.speed.linear.x = 0.0
                    self.speed.angular.z = -0.3  
                else: 
                    self.speed.linear.x = 0.5
                    self.speed.angular.z = 0.0
                    self.pub.publish(self.speed)
            else:
                point_index += 1
            
            time.sleep(0.1) 
        self.stop()
             
    def stop(self):
        self.speed.linear.x = 0.0
        self.speed.angular.z = 0.0
        self.go_to_loc = True
        
    def nothing_found_at_end(self):
         print("nothing found at end")
    
    def is_go_to_loc (self):
        return self.go_to_loc
    
    # def main():
    #     rospy.init_node('aruco_tag_detector', anonymous=True)
    #     AR_detector = ARucoTagDetectionNode()
    #     rospy.spin()

def main(args=None):
    rclpy.init(args=args)
    ar = grid_search_class()
    rclpy.spin(ar)
    ar.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()