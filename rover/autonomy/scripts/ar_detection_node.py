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
        # self.image_topic = "/camera/color/image_raw"
        self.image_topic = "/zed2i/zed_node/rgb/image_rect_color"
        self.info_topic = "/zed2i/zed_node/rgb/camera_info"
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.cam_info_sub = rospy.Subscriber(self.info_topic, CameraInfo, self.info_callback)
        self.state_sub = rospy.Subscriber('rover_state', StateMsg, self.state_callback)
        self.aruco_pub = rospy.Publisher('aruco_node/rover_state', StateMsg, queue_size=10)
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

    def findArucoMarkers(self, img, markerSize=4, totalMarkers=100, draw=True):
        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        key = getattr(aruco, 'DICT_' + str(markerSize) + 'X' + str(markerSize) + "_" + str(totalMarkers))
        arucoDict = aruco.Dictionary_get(key)
        arucoParam = aruco.DetectorParameters_create()
        bboxs, ids, rejected = aruco.detectMarkers(imgGray,arucoDict,parameters=arucoParam)


        if ids is not None:
            print("AR detected!")
            print(ids)
            for i, id in enumerate(ids):
                id = id[0]
                print(id)
                if id in self.curr_aruco_detections: 
                    self.curr_aruco_detections[id] += 1
                    if self.curr_aruco_detections[id] > self.permanent_thresh:
                        self.detected_aruco_ids.append(id)
                
                else:
                    self.curr_aruco_detections[id] = 1


            best_detection = ids[0][0]
            rospy.loginfo(f"An AR tag was detected with the ID {best_detection}")
            bboxs = bboxs[0]
            if draw:
                for bbox, id in zip(bboxs, ids):
                    cv2.rectangle(img, (bbox[0][0], bbox[0][1]) , (bbox[2][0], bbox[2][1]), (0,255,0), 4)
                    # font
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    org = (int(bbox[0][0]), int(bbox[0][1] - 20))
                    fontScale = 1
                    color = (0, 255, 0)
                    thickness = 2
                    img = cv2.putText(img, f"ID: {int(id)}", org, font, 
                                    fontScale, color, thickness, cv2.LINE_AA)
            
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
