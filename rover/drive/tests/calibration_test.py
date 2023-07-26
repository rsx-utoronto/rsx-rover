#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def callback(msg):
    global buttons
    buttons = msg.buttons
    
   

rospy.init_node('calibration_test')
pub = rospy.Publisher('/drive', Twist, queue_size=1)
sub = rospy.Subscriber('/joy', Joy, callback)
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    time = 10
    i = 0
    while i < time:
        move = Twist()
        x = 1
        z = 0
        move.linear.x = 0.1*x
        move.angular.z = 0.1*z
        pub.publish(move)
        i+=1
        if buttons[10] == 1:
            move.linear.x = 0.0
            move.angular.z = 0.0
            pub.publish(move)
            rate.sleep()
        rate.sleep()
