#!/usr/bin/python3

import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from rover.msg import StateMsg
import numpy as np
import os

bridge = CvBridge()

class ARucoTagDetectionNode():

    def __init__(self):
        self.image_topic = "/camera/color/image_raw"
        self.info_topic = "/camera/color/camera_info"
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.cam_info_sub = rospy.Subscriber(self.info_topic, CameraInfo, self.info_callback)
        self.state_sub = rospy.Subscriber('rover_state', StateMsg, self.state_callback)
        self.aruco_pub = rospy.Publisher('rover_state', StateMsg, queue_size=10)
        self.vis_pub = rospy.Publisher('vis/current_aruco_detections', Image, queue_size=10)
        self.bridge = CvBridge()
        self.current_state = StateMsg()
        self.curr_aruco_detections = {}
        self.detected_aruco_ids = []
        self.detect_thresh = 5
        self.permanent_thresh = 10
        self.K = None
        self.D = None
        self.updated_state_msg = StateMsg()

    def image_callback(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image,"bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            # Do we need to undistort?
            self.findArucoMarkers(cv_image)
    
    def info_callback(self, info_msg):

        self.D = np.array(info_msg.D)
        self.K = np.array(info_msg.K)
        self.K = self.K.reshape(3,3)


    def state_callback(self, state_msg):
        
        self.current_state = state_msg

    # from CIRC rules, 4*4_50
    # https://circ.cstag.ca/2022/rules/#autonomy-guidelines:~:text=All%20ArUco%20markers%20will%20be%20from%20the%204x4_50%20dictionary.%20They%20range%20from%20marker%200%20to%2049.

    def findArucoMarkers(self, img, markerSize=4, totalMarkers=50, draw=True):
        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        key = getattr(aruco, 'DICT_' + str(markerSize) + 'X' + str(markerSize) + "_" + str(totalMarkers))
        arucoDict = aruco.Dictionary_get(key)
        arucoParam = aruco.DetectorParameters_create()
        bboxs, ids, rejected = aruco.detectMarkers(imgGray,arucoDict,parameters=arucoParam)

        if ids is not None:

            dists_to_tags = []
            for i, id in enumerate(ids):
                id = id[0]
                if id in self.curr_aruco_detections: 
                    self.curr_aruco_detections[id] += 1
                    if self.curr_aruco_detections[id] > self.permanent_thresh:
                        self.detected_aruco_ids.append(id)
                
                else:
                    self.curr_aruco_detections[id] = 1

                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(bboxs[i], 0.02, self.K,
                                                                                    self.D)
                
                print(rvec)
                print(tvec)
                
                dist = np.linalg.norm(tvec)
                dists_to_tags.append(dist)

            best_detection = ids[np.argmax(np.array(dists_to_tags))]
            rospy.loginfo(f"An AR tag was detected with the ID {best_detection}")
            
            if draw:
                aruco.drawDetectedMarkers(img,bboxs)
            
            img_msg = bridge.cv2_to_imgmsg(img, encoding="passthrough")
            self.vis_pub.publish(img_msg)
            
        self.updated_state_msg = self.current_state

        if ids is not None:
            self.updated_state_msg.AR_TAG_DETECTED = True
            self.updated_state_msg.curr_AR_ID = int(best_detection)
        else:
            self.updated_state_msg.AR_TAG_DETECTED = False
            self.updated_state_msg.curr_AR_ID = -1

        self.aruco_pub.publish(self.updated_state_msg)


def main():
    rospy.init_node('aruco_tag_detector', anonymous=True)

    AR_detector = ARucoTagDetectionNode()

    rospy.spin()


if __name__ == "__main__":
    main()
