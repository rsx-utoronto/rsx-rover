#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class ManualIndicator:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('manual_indicator', anonymous=True)

        # Create a publisher for the manual indicator
        self.pub = rospy.Publisher('/led_light', String, queue_size=10)

        # Subscribe to the joystick input topic
        self.sub = rospy.Subscriber('/software/joy', Joy, self.joy_callback)

        # Set the rate for the loop
        self.rate = rospy.Rate(10)  # 10 Hz

    def joy_callback(self, data):
        if data is not None:
            self.pub.publish("manual")

if __name__ == '__main__':
    try:
        manual_indicator = ManualIndicator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    