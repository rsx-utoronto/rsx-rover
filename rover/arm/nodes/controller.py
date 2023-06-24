#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import _Joy as joy_msg
from std_msgs.msg import String

'''
publishes: state, input
subscribes: joystick angles
rosservice: correction
'''

def getROSJoy(data):
    ''' Gets joystick values from "Joy" Topic

    Assuming using this package:
        http://wiki.ros.org/joy
    
    '''
    pass

if __name__ == "__main__":
 
    try:
        rospy.init_node("controller")
        
        rospy.Subscriber("Joy", joy_msg, getROSJoy)

        statePublisher = rospy.Publisher("arm_states", String, queue_size=10)
        inputPublisher = rospy.Publisher("arm_inputs", String, queue_size=10)
    except rospy.ROSInterruptException:
        pass