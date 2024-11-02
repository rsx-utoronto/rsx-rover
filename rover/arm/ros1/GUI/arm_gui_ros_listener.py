#! /usr/bin/env python3

# Adapted from Week 1 example code

import rospy
from std_msgs.msg import String

def callback(data):
    print("Command received by arm: " + data.data)

def listener():
    rospy.init_node('arm_gui_listener') # Initialize the listener

    rospy.Subscriber('/arm_state', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print("test")
    listener()