#! /usr/bin/env python3
import rospy
from std_msgs.msg import String, Float32MultiArray
from rover.msg import ArmInputs

'''
This code is just to help debug IK messages with 
'''

global curStates
global jointPublisher
global armAngles

def updateStates(data):
    global curState
    curState = data.data

def updateController(data):
    global jointPublisher
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
        print(armAngles)

def updateRealAngles(data):
    global curState
    if curState == "IK":
        global armAngles

        tempList = list(data.data)
        # tempList[0] = tempList[0]
        # tempList[1] = -tempList[1]
        # tempList[4] = -tempList[4]

        armAngles = tempList

        anglesToPublish = Float32MultiArray()
        anglesToPublish.data = armAngles
        jointPublisher.publish(anglesToPublish)

if __name__ == "__main__":
    try:
        rospy.init_node("fake_manual")
        armAngles = [0, 0, 0, 0, 0, 0, 0]

        jointPublisher = rospy.Publisher("arm_curr_pos", Float32MultiArray, queue_size=10)
        rospy.Subscriber("arm_state", String, updateStates)
        rospy.Subscriber("arm_inputs", ArmInputs, updateController)
        rospy.Subscriber("arm_goal_pos", Float32MultiArray, updateRealAngles)

        rospy.spin()
    except Exception as ex:
        print(ex)