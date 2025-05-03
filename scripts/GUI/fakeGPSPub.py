#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
import time
import random
import threading

def aruco_found_publisher():
    aruco_found_pub = rospy.Publisher('aruco_found', Bool, queue_size=10)
    while not rospy.is_shutdown():
        aruco_found_pub.publish(True)
        rospy.loginfo("Published: Aruco Found = True")
        time.sleep(2)
        aruco_found_pub.publish(False)
        rospy.loginfo("Published: Aruco Found = False")
        time.sleep(2)

def aruco_name_publisher():
    aruco_name_pub = rospy.Publisher('aruco_name', String, queue_size=10)
    aruco_names = ["Aruco_1", "Aruco_2", "Aruco_3"]
    while not rospy.is_shutdown():
        for name in aruco_names:
            aruco_name_pub.publish(name)
            rospy.loginfo(f"Published: Aruco Name = {name}")
            time.sleep(1)

def led_colour_publisher():
    led_colour_pub = rospy.Publisher('/led_colour', String, queue_size=10)
    led_colors = ["red", "green", "yellow"]
    while not rospy.is_shutdown():
        for color in led_colors:
            led_colour_pub.publish(color)
            rospy.loginfo(f"Published: LED Colour = {color}")
            time.sleep(3)

def publish_gps_coordinates():
    gps_pub = rospy.Publisher('/calian_gnss/gps', NavSatFix, queue_size=10)
    rate = rospy.Rate(1)
    latitude = 38.4
    longitude = -110.78
    while not rospy.is_shutdown():
        gps_msg = NavSatFix()
        gps_msg.header.stamp = rospy.Time.now()
        gps_msg.header.frame_id = "gps"
        latitude += random.uniform(-0.0001, 0.0001)
        longitude += random.uniform(-0.0001, 0.0001)
        gps_msg.latitude = latitude
        gps_msg.longitude = longitude
        gps_msg.altitude = random.uniform(0, 500)
        rospy.loginfo(f"Publishing GPS coordinates: Latitude: {gps_msg.latitude}, Longitude: {gps_msg.longitude}, Altitude: {gps_msg.altitude}")
        gps_pub.publish(gps_msg)
        rate.sleep()

def status_publisher():
    status_pub = rospy.Publisher('/status', String, queue_size=10)
    statuses = ["Idle", "Processing", "Complete", "Error"]
    while not rospy.is_shutdown():
        status = random.choice(statuses)
        status_pub.publish(status)
        rospy.loginfo(f"Published: Status = {status}")
        time.sleep(5)

if __name__ == '__main__':
    rospy.init_node('test_publishers')
    threading.Thread(target=aruco_found_publisher, daemon=True).start()
    threading.Thread(target=aruco_name_publisher, daemon=True).start()
    threading.Thread(target=led_colour_publisher, daemon=True).start()
    threading.Thread(target=publish_gps_coordinates, daemon=True).start()
    threading.Thread(target=status_publisher, daemon=True).start()
    rospy.spin()
