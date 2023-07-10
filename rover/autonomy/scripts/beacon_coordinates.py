#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import Odometry
import math
import numpy as np

def odom_callback(msg):
    rospy.loginfo("odom_callback")
    global rover_orientation, rover_position
    rover_position = msg.pose.pose.position
    rover_orientation = msg.pose.pose.orientation


def depth_callback(msg):
    global beacon_horizontal_distance
    print("depth_callback")
    if msg.data != 0:
        beacon_depth_in_mm = msg.data
        beacon_depth = beacon_depth_in_mm / float(1000.0)  # converted to metres
        beacon_height = 0.5 # check this value
        if beacon_depth < beacon_height:
            beacon_angle = math.pi / 2
        else:
            beacon_angle = math.asin(beacon_height / beacon_depth)

        beacon_horizontal_distance = math.cos(beacon_angle) * beacon_depth
    else:
        beacon_depth_in_mm = 0
        beacon_horizontal_distance = 0

def calculate_coordinates():
    global beacon_horizontal_distance, rover_position, rover_orientation
    beacon_x = rover_position.x + beacon_horizontal_distance * math.cos(rover_orientation.z)
    beacon_y = rover_position.y + beacon_horizontal_distance * math.sin(rover_orientation.z)

    return beacon_x, beacon_y




rospy.init_node('beacon_gps')
rospy.loginfo("Finding the gps coordinates of the beacon...")

odom_sub = rospy.Subscriber("/odom", Odometry, odom_callback)

depth_sub = rospy.Subscriber('/beacon_spot_depth', Float32, depth_callback)
# rospy.spin()
rate = rospy.Rate(1)

beacon_x, beacon_y = calculate_coordinates()

coordinates_pub = rospy.Publisher("/beacon_coordinates", Float32MultiArray, queue_size=1) 

while not rospy.is_shutdown():
    coordinates = Float32MultiArray()
    coordinates.data = [beacon_x, beacon_y]
    coordinates_pub.publish(coordinates)
    rate.sleep()

