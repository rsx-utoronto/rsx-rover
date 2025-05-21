#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import time

class ManualIndicator:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('manual_indicator', anonymous=True)

        # Create a publisher for the manual indicator
        self.pub = rospy.Publisher('/led_light', String, queue_size=10)

        # Subscribe to the joystick input topic
        self.sub = rospy.Subscriber('/software/joy', Joy, self.joy_callback)

        self.init_time = 0

    def joy_callback(self, data):
        # print("pub manual")
        if abs(self.init_time - time.time()) > 10:
            msg = String()
            msg.data = "manual"
            self.pub.publish(msg)
        self.init_time = time.time()
        
if __name__ == '__main__':
    try:
        manual_indicator = ManualIndicator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    