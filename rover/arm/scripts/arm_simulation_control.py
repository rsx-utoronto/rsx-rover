#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def anglePosition():
            angle1, angle2, angle3, angle4, angle5, angle6 = input("Enter 6 angles (seperated by spaces) in degrees: ").split() # Angles in degrees
            angleArray = list(map(float, [angle1, angle2, angle3, angle4, angle5, angle6])) 
            for i in range(len(angleArray)):
                angleArray[i] = angleArray[i] * math.pi/180 # Conversion from degrees to radians
            return angleArray

if __name__ == "__main__": # if used rosrun on this script then ...
    try:
        rospy.init_node("arm_sim_control") # start node
        jointPublisher = rospy.Publisher("joint_states", JointState, queue_size=10) # refrence to the output topic, you can have multiple in a script

        rate = rospy.Rate(10) # 10 hz
        
        while not rospy.is_shutdown():
            # data to be published
            newJointState = JointState()
            newJointState.header = Header()
            newJointState.header.stamp = rospy.Time.now()
            newJointState.name = ["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6"]
            newJointState.position = anglePosition() # Angles in radians [Joint_1, Joint_2, ....], re-run this script and change the values to see it work.
            jointPublisher.publish(newJointState) # send data to be published
            rate.sleep() # controls loop rate based on what is set in the rate variable 
    except:
        rospy.loginfo("Some Error Has Occured")