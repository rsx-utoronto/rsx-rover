#!/usr/bin/python3


import rospy
from std_msgs.msg import Float32MultiArray

rospy.init_node("goal_pub")

pub = rospy.Publisher("/goal_array", Float32MultiArray, queue_size=10)


goal_pub_arr = Float32MultiArray()
goal_arr = [0, 0, 2, 0, 2, 1, 1.5, 0, 1, 0, 2, 1, 4, 0, 3, 0]
rate = rospy.Rate(1)
count = 0
while not rospy.is_shutdown():
    if count > 2:
        break
    goal_pub_arr.data = goal_arr
    pub.publish(goal_pub_arr)
    count +=1
    rate.sleep()