#!/usr/bin/env python3

import cv2
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np 

def repub_images():
    # super().init_node('imx219_pub')
    node=rclpy.create_node('repub_wrist_cam')
    image_pub= node.create_publisher( Image, "gripper_cam", 10)
    bridge=CvBridge()
    # rate=rospy.Rate(10)

    cap = cv2.VideoCapture("/dev/video0")
    if not cap.isOpened():
        print("Error")
        return
    while rclpy.ok():
        ret,frame = cap.read()
        if not ret:
            print("Error")
            break

        frame = cv2.resize(frame, (240, 135))
        image_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        # image_msg.header.stamp = #rospy.Time.now()
        image_msg.header.stamp=clock.now().to_msg()
        image_pub.publish(image_msg)
        time.sleep(1)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    rclpy.init()
    
    repub_images()