#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Float64
import math

"""
Gets the heading of the magnetic field given by ZED camera
"""

class MagHeading(Node):
    def __init__(self):
        super().__init__('mag_heading_node')
        
        # self.mag_sub = rospy.Subscriber('/zed_node/imu/mag', MagneticField, self.mag_callback)
        self.mag_sub = self.create_subscription(MagneticField, '/zed_node/imu/mag', self.mag_callback, 10)
        # self.mag_heading_pub = rospy.Publisher('/compass', Float64, queue_size=1)
        self.mag_heading_pub = self.create_publisher(Float64, '/compass', 10)
        self.mag_heading = Float64()

    def mag_callback(self, msg):
        self.mag_heading.data = self.get_heading(msg.magnetic_field.x, msg.magnetic_field.y)

    def get_heading(self, x, y):
        heading = 0
        if x != 0:
            heading = math.atan2(y, x)
        return heading

    def run(self):
        # rate = rospy.Rate(10)
        node.create_rate(10)
        
        while rclpy.ok():
            self.mag_heading_pub.publish(self.mag_heading)
            rate.sleep()

if __name__ == '__main__':
    rclpy.init(args=None)
    try:
        mag_heading = MagHeading()
        rclpy.spin(mag_heading)
    except rclpy.exceptions.ROSInterruptException:
        pass    