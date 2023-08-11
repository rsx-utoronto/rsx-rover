#!/usr/bin/env python3
import rospy
import subprocess
import time
from std_msgs.msg import String

def callback(data):
    if data.data == "RECORD":
        command = ["rosbag", "record", "-a"]
        subprocess.run(command)

def main():
    rospy.init_node("recordBag")
    rospy.Subscriber("/gui_node/bagCommand", String, callback)
    while not rospy.is_shutdown():
        time.sleep(0.1)

if __name__ == '__main__':
    main()
