#! /usr/bin/env python3
import rospy
from std_msgs.msg import String, Float32MultiArray
from rover.msg import ArmInputs

'''
This code is just to help debug arm control code when
the real arm isn't plugged in an manual is needed
'''

global curState
global jointPublisher
global armAngles # the angles that fake_manual thinks it's at
global gazebo_on

def updateStates(data):
    ''' Callback function for the /arm_states topic'''
    global curState
    curState = data.data

def updateController(data):
    ''' Callback function for /arm_inputs 

        Recieves the ArmInput ros message and uses the values to change
        the arm angles that fake manual thinks it's at.
    '''
    global jointPublisher
    global realJointPublisher
    global armAngles
    global curState

    if curState == "Manual":
        armAngles[0] += data.l_horizontal
        armAngles[1] += data.l_vertical
        armAngles[2] += data.r_horizontal
        armAngles[3] += data.r_vertical
        armAngles[4] += data.l1 - data.r1
        armAngles[5] += data.l2 - data.r2
        armAngles[6] += data.x - data.o

        anglesToPublish = Float32MultiArray()
        anglesToPublish.data = armAngles
        jointPublisher.publish(anglesToPublish)
        realJointPublisher.publish(anglesToPublish)
        #print(armAngles)

def updateRealAngles(data):
    ''' Callback function for /arm_goal_pos topic

    Idealized version of manual with no saftey
    '''
    global curState
    if curState != "Manual":
        global armAngles

        tempList = list(data.data)
        # tempList[0] = tempList[0]
        # tempList[1] = -tempList[1]
        # tempList[4] = -tempList[4]
        armAngles = tempList

        anglesToPublish = Float32MultiArray()
        anglesToPublish.data = armAngles
        realJointPublisher.publish(anglesToPublish)

def keyboardController(data):
    ''' Callback function for /keyboard_fake_man topic

    Used to pretend to send inputs to fake manual when 
    there isn't a controller. Useful in docker debugging.
    '''
    global curState

    if curState == "Manual":
        newAngles = list(data.data)
        armAngles = newAngles

        anglesToPublish = Float32MultiArray()
        anglesToPublish.data = armAngles
        jointPublisher.publish(anglesToPublish)


if __name__ == "__main__":
    try:
        rospy.init_node("fake_manual")
        armAngles = [0, 0, 0, 0, 0, 0, 0]

        jointPublisher = rospy.Publisher("arm_goal_pos", Float32MultiArray, queue_size=10)
        realJointPublisher = rospy.Publisher("arm_curr_pos", Float32MultiArray, queue_size=10)
        rospy.Subscriber("arm_state", String, updateStates)
        rospy.Subscriber("arm_goal_pos", Float32MultiArray, updateRealAngles)

        curState = "Idle"

        rospy.Subscriber("arm_inputs", ArmInputs, updateController)

        rospy.spin()
    except Exception as ex:
        print(ex)