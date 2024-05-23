#!/usr/bin/env python3

import math
import numpy as np
import roslib
roslib.load_manifest('sensor_msgs')
import sys
import time
import rospy
import cv2
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Empty, EmptyResponse
from rover.msg import StateMsg

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
import os

class aruco_detector:
    def __init__(self):
        self.template_pub = rospy.Publisher("/camera/color/image_raw",Image, queue_size=10)
        # self.template_pub2 = rospy.Publisher("/camera/color/image_raw",Image, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/zed_node/rgb/image_rect_color", Image, self.image_callback)
        path = os.path.dirname(os.path.abspath(__file__))
        template_path = os.path.join(path, 'aruco_F.png')
        self.template = cv2.imread(template_path, cv2.IMREAD_COLOR)
        self.mask = self.make_mask (path, self.template, self.template.shape[0], self.template.shape[1])
        # template1_path = os.path.join(path, 'aruco_shade.png')
        # self.template1 = cv2.imread(template1_path, cv2.IMREAD_COLOR)

        # template2_path = os.path.join(path, 'aruco_shade.png')
        # self.template2 = cv2.imread(template2_path, cv2.IMREAD_COLOR) 
        
        # self.check = False
        self.method =cv2.TM_CCORR_NORMED
        self.method2=cv2.TM_SQDIFF
        #self.method2= cv2.TM_SQDIFF_NORMED
        self.threshold = 0.4 # 0.4
        self.colour = (0,255,0)
        bridge = CvBridge()
        self.ros_image = bridge.cv2_to_imgmsg(self.template, encoding='bgr8')
        self.template_pub.publish(self.ros_image)
    
        # self.ros_image1 = bridge.cv2_to_imgmsg(self.template1, encoding='bgr8')
        # self.ros_image2 = bridge.cv2_to_imgmsg(self.template2, encoding='bgr8')
        # self.template_pub1.publish(self.ros_image1)
    

    def image_callback(self, msg):
        # print("image_callback is working")
        bridge = CvBridge()
        try:
            self.cv_image = bridge.imgmsg_to_cv2(msg, "bgr8") 
        except Exception as e:
            rospy.logerr("Could not convert from '{}' to 'bgr8'".format(msg.encoding))
        else: 
            print("going into detect")
           # print ("template coords [height, width]", self.template.shape)            
            # print ("img coords [height, width]", self.cv_image.shape)
           # print ("shade [height, width]", self.template2.shape)
            
            # TEMPLATE CHANGE
            self.detect_template(self.cv_image, self.cv_image.shape[0], self.cv_image.shape[1])
    

            # self.detect_template(self.cv_image, self.cv_image.shape[0], self.cv_image.shape[1])

    def detect_template(self, img, h, w):
        """
        This function takes as input a template and an image. It searches the image
        for said template and returns the bottom left corner of each of the
        identified templates.

        :param template: The cv2 image of the template we search for
        :param image: The cv2 image we are searching for template occurrences
        :param h: The integer height of the template
        :param w: The integer width of the template
        :return: A list of tuples of the form (a,b), where a is the x coordinate
            and b the y coordinate of the bottom left corner of a found template
        """
        max_max_val = 0
        max_max_loc = (0,0)
        resized_image = cv2.resize(img, (2560,1440), interpolation= cv2.INTER_LINEAR)       
        # initialize box for boundary
        h_box, w_box = h, w

        # tol = 100
            
        # # difference calculations between RGB values
        # rg = np.abs(resized_image[:,:,0]-resized_image[:,:,1])
        # rb = np.abs(resized_image[:,:,0]-resized_image[:,:,2])        
        # gb = np.abs(resized_image[:,:,1]-resized_image[:,:,2])

        # # all values outside of greyscale will be made red    
        # mask = (rg > tol) | (rb > tol) | (gb > tol)
        # resized_image[mask]=[0,0,255]
    
        for i in range (2, 7):  # range to resize image
            new_h = int(h*i)
            new_w = int(w*i)
            resized_image= cv2.resize(self.cv_image, (new_w,new_h), interpolation= cv2.INTER_LINEAR) 
            result = cv2.matchTemplate(resized_image, self.template, self.method2, None, self.mask)

            result_normalized = cv2.normalize(result, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result_normalized)
            print (min_val, max_val)

        #     if max_val > self.threshold:
        #         if max_val > max_max_val:
        #             max_max_loc = max_loc
        #             max_max_val = max_val

        #             if max_val > 0.45:
        #                 self.check = True
        #                 print ("Using shaded template")

        # if max_max_val > self.threshold:
        #     print("max_max_loc", max_max_loc, "max_max_val", max_max_val)
        #     found = []
        #     result_normalized[max_max_loc[1] - h // 2 : max_max_loc[1] + h // 2 + 1, max_max_loc[0] - w // 2 : max_max_loc[0] + w // 2 + 1] = 0
        #     found.append(max_max_loc)
        #     print("")
        #     print("FOUND STUFF")
        #     print("")
        #     self.draw_match_boundaries(resized_image,h_box,w_box,found)
        # else:
        #     # found = [(0,0)]
        #     # self.draw_match_boundaries(resized_image,h_box,w_box,found)
        #     print("")
        #     print("NOTHING DETECTED")
        #     print("")

    def draw_match_boundaries(self,image,h,w,locs):
        """
        This function draws a box around at the specified location of the image,
        with a height h and width w. The now edited image gets saved to fileName.

        :param image: The cv2 image we are drawing bounding boxes on
        :param locations: A list of locations (represented as a tuple (x,y)) to draw
            the boxes
        :param fileName: A str of the file to save the edited image to
        :param h: The integer height of the boxes
        :param w: The integer width of the boxes
        """
        print ("locations", locs)



        # for each location we will draglobal cv_imagew a bounding box of given height and width
        for location in locs:
            x,y = location
            image = cv2.rectangle(image,(x,y), (x+w+1, y+h+1), self.colour)
            print (w,h,(x+w+1, y+h+1))
            cv2.imshow("result_normalized", image)
            cv2.imwrite("aruco_phone1_detected.jpg", image)
        pub = rospy.Publisher('aruco_detect', Image, queue_size= 10)
        bridge = CvBridge()
        im_resize = cv2.resize(image, (640, 360), interpolation= cv2.INTER_LINEAR)
        ros_image = bridge.cv2_to_imgmsg(im_resize, encoding='bgr8')
        pub.publish(ros_image)
        print("draw match boundaries is bounding")
 

    def make_mask (self, path, image, h, w):
        #this function takes image and template dimensions, checks if theyre equal, resizes if they're not equal, the it masks the templ
        mask = np.zeros((h, w), dtype=np.uint8)

        mask[44:140, 140:748] = 255
        mask[540:638, 140:748] = 255
        mask[140:550, 140:230] = 255
        mask[140:550, 640:748] = 255
        mask_conv = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        print ("make_mask made")
        cv2.imwrite(os.path.join(path, 'CHECK.png'), mask_conv)
        return mask_conv

if __name__ == "__main__":
    while not rospy.is_shutdown():
        rospy.init_node('aruco_location')   
        rospy.loginfo("Node 'image_listener' initialized")
        cv2.namedWindow("view")
        ad = aruco_detector()
        rospy.spin()