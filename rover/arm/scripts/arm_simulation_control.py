#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def anglePosition():
    '''
    Prompts the user to enter 6 angles in degrees and returns an array of size 6. 
    '''
    angle1, angle2, angle3, angle4, angle5, angle6 = input("Enter 6 angles (seperated by spaces) in degrees: ").split() # Angles in degrees
    angleArray = list(map(float, [angle1, angle2, angle3, angle4, angle5, angle6])) 
    for i in range(len(angleArray)):
        angleArray[i] = angleArray[i] * math.pi/180 # Conversion from degrees to radians
    return angleArray

def startJointPublisher():
    '''
    Initiates arm_sim_control, uses rospy.Publisher to take in the joint states and returns jointPublisher
    '''
    # rospy.init_node("arm_sim_control") # start node
    jointPublisher = rospy.Publisher("joint_states", JointState, queue_size=10) # refrence to the output topic, you can have multiple in a script
    return jointPublisher

def runNewJointState(jointPublisherData):
    '''
    Publishes the newJointState header, stamp, name, and position in a continuous while loop. 
    '''
    rate = rospy.Rate(10) # 10 hz
    while not rospy.is_shutdown():
        # data to be published
        newJointState = JointState()
        newJointState.header = Header()
        newJointState.header.stamp = rospy.Time.now()
        newJointState.name = ["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6"]
        newJointState.position = anglePosition() # Angles in radians [Joint_1, Joint_2, ....], re-run this script and change the values to see it work.
        jointPublisherData.publish(newJointState) # send data to be published
        rate.sleep() # controls loop rate based on what is set in the rate variable 

def runNewJointState2(jointPublisherData, angles):
    '''
    Publishes the newJointState header, stamp, name, and position in a continuous while loop. 
    '''

    # data to be published
    newJointState = JointState()
    newJointState.header = Header()
    newJointState.header.stamp = rospy.Time.now()
    newJointState.name = ["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6"]
    newJointState.position = angles # Angles in radians [Joint_1, Joint_2, ....], re-run this script and change the values to see it work.
    jointPublisherData.publish(newJointState) # send data to be published

if __name__ == "__main__": # if used rosrun on this script then ...
    try:
        jointPublisher = startJointPublisher()
        runNewJointState(jointPublisher)
        
    except:
        rospy.loginfo("Some Error Has Occured")