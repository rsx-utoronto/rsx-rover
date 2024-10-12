#!/usr/bin/env python3
import rospy
import random
from sensor_msgs.msg import NavSatFix
from PyQt5.QtCore import QThread
import time

class FakeGPSPublisher(QThread):
    def __init__(self):
        super(FakeGPSPublisher, self).__init__()
        self.latitude = 40.7608  # Starting latitude (e.g., in Utah, near Salt Lake City)
        self.longitude = -111.8910  # Starting longitude (e.g., in Utah, near Salt Lake City)
        self.altitude = 1500  # Approximate altitude (adjust as needed)
        self.publisher = rospy.Publisher("/gps/fix", NavSatFix, queue_size=10)
        self.rate = rospy.Rate(1)  # Limit to 1 Hz (1 message per second)

    def move_gps(self):
        """
        Simulate movement by adding a small random change to the latitude and longitude.
        This ensures the data is realistic and small changes are made for each iteration.
        """
        move_distance = 0.000045  # ~5 meters in GPS coordinates
        direction = random.choice(["north", "south", "east", "west"])
        
        if direction == "north":
            self.latitude += move_distance
        elif direction == "south":
            self.latitude -= move_distance
        elif direction == "east":
            self.longitude += move_distance
        elif direction == "west":
            self.longitude -= move_distance

    def run(self):
        """
        The main loop that publishes fake GPS data at a controlled rate.
        """
        while not rospy.is_shutdown():
            self.move_gps()  # Simulate movement

            # Create the GPS message
            gps_msg = NavSatFix()
            gps_msg.latitude = self.latitude
            gps_msg.longitude = self.longitude
            gps_msg.altitude = self.altitude
            gps_msg.header.stamp = rospy.Time.now()

            # Publish the GPS message
            self.publisher.publish(gps_msg)

            rospy.loginfo(f"Published GPS data: Lat={self.latitude}, Lon={self.longitude}")

            # Use rospy.Rate to control the message publishing frequency (1 Hz = 1 message per second)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('fake_gps_publisher', anonymous=False)
    
    # Start the fake GPS publisher in a separate thread
    fake_gps_publisher = FakeGPSPublisher()
    fake_gps_publisher.start()
    
    rospy.spin()