#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray

rospy.init_node("test")

while not rospy.is_shutdown():
    try:
        pub = rospy.Publisher("ik_angles", Float32MultiArray, queue_size=10)
        rate = rospy.Rate(10)
        msg = Float32MultiArray()
        msg.data = [1,1,1,1]
        pub.publish(msg)
        rate.sleep()
    except:
        pass
