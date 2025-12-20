#!/usr/bin/python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time


rclpy.init(args=None)
node=rclpy.create_node('goal_publisher')
pub= node.create_publisher(Float32MultiArray, '/goal_array', 10)

goal_pub_arr = Float32MultiArray()
# goal_arr = [0, 0, 2, 0, 2, 1, 1.5, 0, 1, 0, 2, 1, 4, 0, 3, 0]
goal_arr = [float(x) for x in [0, 0, 2, 0, 2, 1, 1.5, 0, 1, 0, 2, 1, 4, 0, 3, 0]]
# time.sleep(1)
count = 0
while rclpy.ok():
    if count > 2:
        break
    goal_pub_arr.data = goal_arr
    pub.publish(goal_pub_arr)
    count +=1
    time.sleep(1)