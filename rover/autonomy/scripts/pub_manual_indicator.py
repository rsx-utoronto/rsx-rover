#!/usr/bin/env python3

from pathlib import Path
from matplotlib import axes

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import subprocess

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
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.find_usb_script_path = Path("~/rover_ws/src/rsx-rover/scripts/utils/gen/find_usb.sh").expanduser()
        # self.sub = self.create_subscription(Joy, '/software/joy', self.joy_callback, 10)
        self.init_time = 0

    def timer_callback(self):
        res = subprocess.run(["bash", str(self.find_usb_script_path)], capture_output=True, text=True)
        usb_list = res.stdout.strip().split('\n')
        for item in usb_list:
            if "Sony_Interactive_Entertainment_Wireless_Controller" in item:
                msg = String()
                msg.data = "manual"
                self.pub.publish(msg)
                break

    def joy_callback(self, data):
        # print("pub manual")
        if data is not None:
            # Check if the joystick is in manual mode
            if not (
                data.axes[0] == 0.0 and
                data.axes[1] == 0.0 and
                data.axes[2] == 1.0 and
                data.axes[3] == 0.0 and
                data.axes[4] == 0.0 and
                data.axes[5] == 1.0 and
                data.axes[6] == 0.0 and
                data.axes[7] == 0.0 and
                data.buttons[0] == 0 and
                data.buttons[1] == 0 and
                data.buttons[2] == 0 and
                data.buttons[3] == 0 and
                data.buttons[4] == 0 and
                data.buttons[5] == 0 and
                data.buttons[6] == 0 and
                data.buttons[7] == 0 and
                data.buttons[8] == 0 and
                data.buttons[9] == 0 and
                data.buttons[10] == 0 and
                data.buttons[11] == 0 and
                data.buttons[12] == 0 
                ):
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

    