#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

def odom_callback(msg):
    global x, y, heading
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    heading = ToEulerAngles(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)[2]
    # heading is in radians


    if __name__ == '__main__':
    x = None
    y = None
    heading = None
    target_x = None
    target_y = None
    try:
        straight_line_approach(1, 1) # change linear and angular velocities
    except rospy.ROSInterruptException:
        pass
