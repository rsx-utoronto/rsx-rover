#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

import argparse
import os

def gps_callback(gps_msg):
    # save GPS message to text file
    print("GPS message received")
    os.system("rostopic echo  > gps.txt")


def main():
    rclpy.init()
    node=rclpy.create_node('gps_to_txt')
    

    # gps rostopic name
    gps_topic = "/fix" 
    node.create_subscription(NavSatFix,gps_topic,  gps_callback, 10)
    rclpy.spin(node)

if __name__ == '__main__':
    main()





