#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

import time

class ManualIndicator(Node):
    def __init__(self):
        # Initialize the ROS node
        super().__init__('manual_indicator_node')
        

        # Create a publisher for the manual indicator
        # self.pub = rospy.Publisher('/led_light', String, queue_size=10)

        # # Subscribe to the joystick input topic
        # self.sub = rospy.Subscriber('/software/joy', Joy, self.joy_callback)

        self.pub = self.create_publisher(String, '/led_light', 10)
        self.sub = self.create_subscription(Joy, '/software/joy', self.joy_callback, 10)
        self.init_time = 0

    def joy_callback(self, data):
        # print("pub manual")
        if data is not None:
            # Check if the joystick is in manual mode
            if abs(self.init_time - time.time()) > 1:
                msg = String()
                msg.data = "manual"
                self.pub.publish(msg)
            self.init_time = time.time()
        
if __name__ == '__main__':
    rclpy.init(args=None)
    try:
        manual_indicator = ManualIndicator()
        rclpy.spin(manual_indicator)
    except rclpy.exceptions.ROSInterruptException:
        pass

    