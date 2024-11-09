#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
import random

def publish_gps_coordinates():
    # Initialize the ROS node
    rospy.init_node('gps_publisher', anonymous=True)
    
    # Create a publisher for the /gps/fix topic
    gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
    
    # Set the rate at which to publish (e.g., 1 Hz)
    rate = rospy.Rate(1)  # 1 Hz
    
    # Starting position for simulated GPS coordinates (e.g., near New York City)
    latitude = 40.7128  # Initial latitude
    longitude = -74.0060  # Initial longitude

    while not rospy.is_shutdown():
        # Create a NavSatFix message
        gps_msg = NavSatFix()
        gps_msg.header.stamp = rospy.Time.now()
        gps_msg.header.frame_id = "gps"

        # Adjust latitude and longitude slightly
        latitude += random.uniform(-0.01, 0.01)  # Move latitude slightly
        longitude += random.uniform(-0.01, 0.01)  # Move longitude slightly

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
    try:
        publish_gps_coordinates()
    except rospy.ROSInterruptException:
        pass
