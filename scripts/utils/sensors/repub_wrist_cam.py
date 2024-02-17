#!/usr/bin/env python3

import cv2
import time
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np 

def repub_images():
    rospy.init_node("imx219_pub")
    image_pub= rospy.Publisher("gripper_cam", Image, queue_size=10)
    bridge=CvBridge()
    rate=rospy.Rate(10)

    cap = cv2.VideoCapture("/dev/video0")
    if not cap.isOpened():
        print("Error")
        return
    while not rospy.is_shutdown():
        ret,frame = cap.read()
        if not ret:
            print("Error")
            break

        frame = cv2.resize(frame, (240, 135))
        image_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = rospy.Time.now()
        image_pub.publish(image_msg)
        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    repub_images()