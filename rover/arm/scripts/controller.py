#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from rover.msg import ArmInputs
from rover.srv import Corrections

'''
publishes: arm_state, arm_inputs (joystick value)
subscribes: Joy
rosservice: correction
'''

def getROSJoy(data):    
    ''' Gets joystick values from "Joy" Topic

    Assuming using this package:
        http://wiki.ros.org/joy
    
    
    Paramters
    ---------
    data
        of type sensor_msgs.msg.Joy and automatically 
    '''
    rawAxes = data.axes
    rawButtons = data.buttons

def updateCorrections():
    ''' Update the Correction Values the IK node is using

    Paramters
    ---------

    Returns
    -------
    '''

    try:
        correctionService = rospy.ServiceProxy('update_ik_corrections', Corrections)
        serviceResponse = correctionService() # input data into function to send to service, response contains boolean if succesfully updated or not
    except rospy.ServiceException as ex:
        print("Service call failed: %s"%ex)
 
if __name__ == "__main__":
 
    try:
        rospy.init_node("controller")
        
        rospy.Subscriber("Joy", Joy, getROSJoy)

        statePublisher = rospy.Publisher("arm_state", String, queue_size=10)
        inputPublisher = rospy.Publisher("arm_inputs", ArmInputs, queue_size=10)
        while not rospy.is_shutdown():
            newAngles = ArmInputs()
            newAngles.r_right = 1
            newAngles.r_down = 2
            inputPublisher.publish(newAngles)

        
    except rospy.ROSInterruptException:
        pass