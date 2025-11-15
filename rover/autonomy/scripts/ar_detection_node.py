#!/usr/bin/python3

import rclpy 
from rclpy.node import Node
import cv2
import time
import cv2.aruco as aruco
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import os
import yaml
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

file_path = os.path.join(os.path.dirname(__file__), "sm_config.yaml")


with open(file_path, "r") as f:
    sm_config = yaml.safe_load(f)

bridge = CvBridge()

class ARucoTagDetectionNode(Node):

    def __init__(self):
        super().__init__('aruco_tag_detector')
        self.bridge = CvBridge()
        self.curr_state = None
        # self.image_topic = "/camera/color/image_raw"
        
        if sm_config.get("realsense_detection"):
            self.image_topic = sm_config.get("realsense_detection_image_topic")
            self.info_topic = sm_config.get("realsense_detection_info_topic")
            
        else:
            self.image_topic = sm_config.get("zed_detection_image_topic") 
            print("zed topic:", self.image_topic)
            self.info_topic = sm_config.get("zed_detection_info_topic")
       
        self.state_topic = "state"
        # self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.image_sub = self.create_subscription(Image,self.image_topic, self.image_callback,10)
        self.cam_info_sub = self.create_subscription( CameraInfo,self.info_topic, self.info_callback, 10)
        self.state_sub = self.create_subscription( String,self.state_topic, self.state_callback, 10)
        #self.state_sub = rospy.Subscriber('rover_state', StateMsg, self.state_callback)
        #self.aruco_pub = rospy.Publisher('aruco_node/rover_state', StateMsg, queue_size=10)
        #self.scanned_pub = rospy.Publisher('aruco_scanned_node/rover_state', StateMsg, queue_size=10)
        # self.aruco_pub = rospy.Publisher('aruco_found', Bool, queue_size=1)
        self.aruco_pub = self.create_publisher(Bool,'aruco_found', 10)
        self.vis_pub = self.create_publisher(Image, 'vis/current_aruco_detections', 10)
        self.bbox_pub = self.create_publisher(Float64MultiArray,'aruco_node/bbox', 10)
    
        t = time.time()
        
        while (time.time() - t) < 2:
            #print("Passing time") 
            pass
        #self.current_state = StateMsg()
        self.curr_aruco_detections = {}
        self.detected_aruco_ids = []
        self.aruco_locations = []
        self.detect_thresh = 5
        self.permanent_thresh = 10
        self.K = None
        self.D = None
        #self.updated_state_msg = StateMsg()
        #self.scanned_state_smg = StateMsg()
        self.found = False

    def image_callback(self, ros_image):
        print("in image callback")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image,"bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            # Do we need to undistort?    
         
            if self.curr_state == "AR1" or self.curr_state == "AR2" or self.curr_state == "AR3":
                print("calling findArucoMarkers")
                self.findArucoMarkers(cv_image)
    
    def info_callback(self, info_msg):
        print("in info callback")
        self.D = np.array(info_msg.d)
        self.K = np.array(info_msg.k)
        self.K = self.K.reshape(3,3)

    def state_callback(self, state):
        print("in state callback")
        self.curr_state = state.data


    #def state_callback(self, state_msg):
    #    
    #    self.current_state = state_msg

    # from CIRC rules, 4*4_50
    # https://circ.cstag.ca/2022/rules/#autonomy-guidelines:~:text=All%20ArUco%20markers%20will%20be%20from%20the%204x4_50%20dictionary.%20They%20range%20from%20marker%200%20to%2049.

    def findArucoMarkers(self, img, markerSize=4, totalMarkers=100, draw=True):
        print("ar_detection_node: Finding Aruco Markers")
        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        key = getattr(aruco, 'DICT_' + str(markerSize) + 'X' + str(markerSize) + "_" + str(totalMarkers))
        arucoDict = aruco.getPredefinedDictionary(key)
        arucoParam = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(arucoDict, arucoParam)
        bboxs, ids, rejected = detector.detectMarkers(imgGray)
        # bboxs, ids, rejected = aruco.detectMarkers(imgGray,arucoDict,parameters=arucoParam)
        # print("bbox", bboxs)
        if ids is not None:
            print("ar_detection_node: AR detected!")
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
            self.get_logger().info(f"ar_detection_node: An AR tag was detected with the ID {best_detection}")
            #self.scanned_state_smg.AR_SCANNED = True
            bboxs = bboxs[0]
            
            if draw:
                for bbox, id in zip(bboxs, ids):
                    print(bbox)
                    print(type(bbox[0,0]))

                    cv2.rectangle(img, (int(bbox[0,0]), int(bbox[0,1])) , (int(bbox[2,0]), int(bbox[2,1])), (0,255,0), 4)
                    # font

                    print(bbox[0][0],bbox[0][1],bbox[2][0],bbox[2][1])
                    self.array=[bbox[0,0], bbox[0,1], bbox[2,0], bbox[0,1], bbox[0,0], bbox[2,1], bbox[2,0], bbox[2,1]]
                    data = Float64MultiArray(data=self.array)
                    
                    self.bbox_pub.publish(data)
                    
                    # first two numbers are top left corner, second two are bottom right corner
                    print("ar_detection_node: here are the coords from jack's code", self.array[0], self.array[1], self.array[6], self.array[7])
                    print((self.array[6]-self.array[0])*(self.array[7]-self.array[1]))
                    
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    org = (int(bbox[0,0]), int(bbox[0,1] - 20))
                    fontScale = 1
                    color = (0, 255, 0)
                    thickness = 2
                    img = cv2.putText(img, f"ID: {int(id)}", org, font, 
                                    fontScale, color, thickness, cv2.LINE_AA)
            
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")
            self.vis_pub.publish(img_msg)
            
        #self.updated_state_msg = self.current_state

        if ids is not None:
            print(self.aruco_locations)
            #self.updated_state_msg.AR_TAG_DETECTED = True
            #self.updated_state_msg.curr_AR_ID = int(best_detection)

            # Transform into a goal in the odom frame

            # lookup baselink to camera link transform 
            # lookup baselink to odom transform 
            # transform the 4x4 pose to the odom frame and publish below

            #self.scanned_state_smg.AR_SCANNED = False
            self.found = True
        else:
            #self.updated_state_msg.AR_TAG_DETECTED = False
            #self.updated_state_msg.curr_AR_ID = -1
            self.found = False

        #self.aruco_pub.publish(self.updated_state_msg)
        # self.aruco_pub.publish(self.found)
        self.aruco_pub.publish(Bool(data=self.found))
        #self.scanned_pub.publish(self.scanned_state_smg)
     
    def is_found(self):
        return self.found

def main(args=None):
    rclpy.init(args=args)
    node = ARucoTagDetectionNode()
    node.curr_state = "AR1"
    rclpy.spin(node)
    print("spinning node")
    node.destroy_node()
    rclpy.shutdown()
    print("shut down")
    # rospy.init_node('aruco_tag_detector', anonymous=True)
    # AR_detector = ARucoTagDetectionNode()
    
    # rospy.spin()

if __name__ == "__main__":
    main()
    