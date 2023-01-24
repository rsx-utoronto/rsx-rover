#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

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
            newJointState.position = [0, 0, 1, 1, 1, 1] # Angles in radians [Joint_1, Joint_2, ....]

            jointPublisher.publish(newJointState) # send data to be published

            rate.sleep() # controls loop rate based on what is set in the rate variable 
    except:
        rospy.loginfo("Some Error Has Occured")