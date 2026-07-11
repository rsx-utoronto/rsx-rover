#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import threading
import subprocess
import os

class CheckZed(Node):
    def __init__(self):
        super().__init__('check_zed')
        self.dead = False
        self.timer = threading.Timer(5, self.timeout)
        # self.path_to_zed_dies = 'zed_dies' #/home/rsx/rover_ws/src/rsx-rover/scripts/zed_dies'
        # self.path_to_zed_dies = os.path.join(os.path.dirname(__file__), 'scripts', 'zed_dies')
        self.path_to_zed_dies = '/ros_ws/rsx-rover/scripts/zed_dies'
        self.create_subscription(
            Image,
            '/zed_node/rgb/image_rect_color',
            self.callback,
            10
        )
        self.timer.start()

    def callback(self, msg):
        self.timer.cancel()
        self.timer = threading.Timer(5, self.timeout)
        self.timer.start()

    def timeout(self):
        # rospy.logwarn("ZED DIED")
        rclpy.logging.get_logger('check_zed').warn("ZED DIED")
        self.dead = True
        try:
            subprocess.run([self.path_to_zed_dies], check = True)
        except subprocess.CalledProcessError as e:
            # rospy.logerr(f"Failed to run zed_dies")
            rclpy.logging.get_logger('check_zed').error(f"Failed to run zed_dies: {e}")

if __name__ == "__main__":
    rclpy.init(args=None)
    
    check_zed = CheckZed()
    rclpy.spin(check_zed)