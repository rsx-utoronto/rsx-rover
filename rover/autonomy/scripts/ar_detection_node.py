#!/usr/bin/python3

import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import os

bridge = CvBridge()

def image_callback(ros_image):
    global bridge
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image,"bgr8")
    except CvBridgeError as e:
        print("image_callback conversion error")
        print(e)
    else:
        findArucoMarkers(cv_image)

# from CIRC rules, 4*4_50
# https://circ.cstag.ca/2022/rules/#autonomy-guidelines:~:text=All%20ArUco%20markers%20will%20be%20from%20the%204x4_50%20dictionary.%20They%20range%20from%20marker%200%20to%2049.

def findArucoMarkers(img, markerSize=4, totalMarkers=50, draw=True):
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(cv2.aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')

    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(dictionary, parameters)
    bboxs, ids, rejected = detector.detectMarkers(imgGray)

    print(ids)
    if draw:
        aruco.drawDetectedMarkers(img,bboxs)


def main():
    rospy.init_node('image_converter',anonymous=True)
    # set to the right camera for now, can also set left camera
    image_topic = "/camera/color/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback)

    # Spin until ctrl + c
    rospy.spin()

    ### this code is for webcam 
    # cap=cv2.VideoCapture(0)

    # while True:
    #     sccuess, img = cap.read()
    #     findArucoMarkers(img)
    #     cv2.imshow("Image",img)
    #     cv2.waitKey(1)


if __name__ == "__main__":
    main()
