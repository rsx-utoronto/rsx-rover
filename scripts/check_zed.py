#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import threading
import subprocess

class CheckZed:
    def __init__(self):
        self.dead = False
        self.timer = threading.Timer(5, self.timeout)
        self.path_to_zed_dies = '/home/rsx/rover_ws/src/rsx-rover/scripts/zed_dies'
        rospy.init_node('check_zed', anonymous=True)
        rospy.Subscriber("/zed_node/rgb/image_rect_color", Image, self.callback)
        self.timer.start()

    def callback(self, msg):
        self.timer.cancel()
        self.timer = threading.Timer(5, self.timeout)
        self.timer.start()

    def timeout(self):
        rospy.logwarn("ZED DIED")
        self.dead = True
        try:
            subprocess.run([self.path_to_zed_dies], check = True)
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Failed to run zed_dies")

if __name__ == "__main__":
    check_zed = CheckZed()
    rospy.spin()