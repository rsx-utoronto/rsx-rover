#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from rover.msg import ArmInputs
#from rover.srv import Corrections

'''
publishes: arm_state, arm_inputs (joystick value)
subscribes: Joy
rosservice: correction
'''

global state
state = "Idle"

def getROSJoy(data):    
    ''' Gets joystick values from "Joy" Topic

    Assuming using this package:
        http://wiki.ros.org/joy
    
    
    Paramters
    ---------
    data
        of type sensor_msgs.msg.Joy and automatically 
    '''
    global state

    rawAxes = data.axes
    rawButtons = data.buttons
    values = ArmInputs()
    values.l_horizontal = rawAxes[0]
    values.l_vertical = rawAxes[1]
    values.r_horizontal = rawAxes[3]
    values.r_vertical = rawAxes[4]
    values.l1r1 = rawButtons[4] - rawButtons[5]
    values.l2r2 = -0.5*(rawAxes[2] - rawAxes[5])
    values.xo = rawButtons[0] - rawButtons[1]
    values.ps = rawButtons[10]

    if (not (rawAxes[0] or rawAxes[1] or rawAxes[3] or rawAxes[4])) and rawAxes[2] == 1 and rawAxes[5] == 1:
        
        if rawAxes[7] == -1:
            state = "Idle"
        
        elif rawAxes[7] == 1:
            state = "Setup"
        
        elif rawAxes[6] == 1:
            state = "Manual"

        elif rawAxes[6] == -1:
            state = "IK"

    print(state)
    statePublisher.publish(state)

    if state != "Idle" and state != "Setup":
        print(values)
        inputPublisher.publish(values)



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
        
        rospy.Subscriber("joy", Joy, getROSJoy)

        statePublisher = rospy.Publisher("arm_state", String, queue_size=10)
        inputPublisher = rospy.Publisher("arm_inputs", ArmInputs, queue_size=10)

        rospy.spin()

        
    except rospy.ROSInterruptException:
        pass