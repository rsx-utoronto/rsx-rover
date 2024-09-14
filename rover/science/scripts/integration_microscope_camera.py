#! /usr/bin/env python3

import rospy 
from sensor_msgs.msg import Image 
import cv2 
from cv_bridge import CvBridge 
import math
import numpy as np

def capture_image_from_gemini():
    return gemini_api.get_image() 

def cv_ros_cv():
    rospy.init_node('image_publisher')
    image_pub = rospy.Publisher('/camera/image', Image, queue_size=10)
    bridge = CvBridge()
    
    while not rospy.is_shutdown():
        # Step 1: Capture Image from Gemini 2 API 
        gemini_image = capture_image_from_gemini()
        # Step 2: Convert OpenCV image to ROS Image message using CvBridge
        ros_image = bridge.cv2_to_imgmsg(gemini_image, encoding="bgr8")
        # Step 3: Publish the ROS image topic
        image_pub.publish(ros_image)
        rospy.sleep(0.1)
    

if __name__ == '__main__':
        cv_ros_cv()
    
    