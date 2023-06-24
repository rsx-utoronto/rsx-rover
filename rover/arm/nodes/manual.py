#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy as joy_msg

'''
publishes: error messages
subscribes: Input, state
'''

def updateState(data):
    pass

def getInputs(data):
    pass

if __name__ == "__main__":

    try:
        rospy.init_node("manual_and_setup")
    except rospy.ROSInterruptException:
        pass