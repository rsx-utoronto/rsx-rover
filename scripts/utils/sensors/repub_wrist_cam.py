#!/usr/bin/env python3

import cv2
import time
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np 

def repub_images():
    rospy.init_node("cam_pub")
    image_pub= rospy.Publisher("wrist_cam", Image, queue_size=10)
    bridge=CvBridge()
    rate=rospy.Rate(10)

    cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)540, height=(int)540,format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink")
    if not cap.isOpened():
        print("Error")
        return
    while not rospy.is_shutdown():
        ret,frame = cap.read()
        if not ret:
            print("Error")
            break

        frame = cv2.undistort(frame, K, D)
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        frame = cv2.resize(frame, (960, 540))
        image_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = rospy.Time.now()
        image_pub.publish(image_msg)


        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    repub_images()