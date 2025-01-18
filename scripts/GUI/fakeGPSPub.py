#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
import time
import random
import threading

def aruco_found_publisher():
    # Publisher for the 'aruco_found' topic (Bool)
    aruco_found_pub = rospy.Publisher('aruco_found', Bool, queue_size=10)

    # Alternate between True and False every 2 seconds
    while not rospy.is_shutdown():
        aruco_found_pub.publish(True)
        rospy.loginfo("Published: Aruco Found = True")
        time.sleep(2)
        aruco_found_pub.publish(False)
        rospy.loginfo("Published: Aruco Found = False")
        time.sleep(2)

def aruco_name_publisher():
    # Publisher for the 'aruco_name' topic (String)
    aruco_name_pub = rospy.Publisher('aruco_name', String, queue_size=10)
    aruco_names = ["Aruco_1", "Aruco_2", "Aruco_3"]

    # Publish a new ArUco name every second
    while not rospy.is_shutdown():
        for name in aruco_names:
            aruco_name_pub.publish(name)
            rospy.loginfo(f"Published: Aruco Name = {name}")
            time.sleep(1)

def led_colour_publisher():
    # Publisher for the '/led_colour' topic (String)
    led_colour_pub = rospy.Publisher('/led_colour', String, queue_size=10)
    led_colors = ["red", "green", "yellow"]

    # Publish a new LED color every 3 seconds
    while not rospy.is_shutdown():
        for color in led_colors:
            led_colour_pub.publish(color)
            rospy.loginfo(f"Published: LED Colour = {color}")
            time.sleep(3)

def publish_gps_coordinates():
    # Create a publisher for the /calian_gnss/gps topic
    gps_pub = rospy.Publisher('/calian_gnss/gps', NavSatFix, queue_size=10)
    
    # Set the rate at which to publish (e.g., 1 Hz)
    rate = rospy.Rate(1)  # 1 Hz
    
    # Starting position for simulated GPS coordinates (e.g., near New York City)
    latitude = 38.4  # Initial latitude 38.4,-110.78
    longitude = -110.78  # Initial longitude

    while not rospy.is_shutdown():
        # Create a NavSatFix message
        gps_msg = NavSatFix()
        gps_msg.header.stamp = rospy.Time.now()
        gps_msg.header.frame_id = "gps"

        # Adjust latitude and longitude slightly
        latitude += random.uniform(-0.0001, 0.0001)  # Move latitude slightly
        longitude += random.uniform(-0.0001, 0.0001)  # Move longitude slightly

        # Assign the modified values to the message
        gps_msg.latitude = latitude
        gps_msg.longitude = longitude
        gps_msg.altitude = random.uniform(0, 500)  # Simulate altitude (meters)

        # Log the GPS coordinates for debugging
        rospy.loginfo(f"Publishing GPS coordinates: Latitude: {gps_msg.latitude}, Longitude: {gps_msg.longitude}, Altitude: {gps_msg.altitude}")

        # Publish the message
        gps_pub.publish(gps_msg)
        
        # Wait for the next cycle
        rate.sleep()

if __name__ == '__main__':
    # Initialize the ROS node only once in the main thread
    rospy.init_node('test_publishers')

    # Run each publisher in a separate thread
    threading.Thread(target=aruco_found_publisher, daemon=True).start()
    threading.Thread(target=aruco_name_publisher, daemon=True).start()
    threading.Thread(target=led_colour_publisher, daemon=True).start()
    threading.Thread(target=publish_gps_coordinates, daemon=True).start()

    rospy.spin()
