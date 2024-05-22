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
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/zed_node/rgb/image_rect_color", Image, self.image_callback)
        path = os.path.dirname(os.path.abspath(__file__))
        template_path = os.path.join(path, 'aruco_20.png')
        self.template = cv2.imread(template_path, cv2.IMREAD_COLOR)
        self.method = cv2.TM_CCOEFF_NORMED
        self.threshold = 0.4
        self.colour = (0,255,0)
        bridge = CvBridge()
        self.ros_image = bridge.cv2_to_imgmsg(self.template, encoding='bgr8')


        max_max_val = 0
        max_max_loc = (0,0)
        resized_image = cv2.resize(img, (1280,720), interpolation= cv2.INTER_LINEAR)
        tol = 20
        rg = np.abs(resized_image[:,:,0]-resized_image[:,:,1])
        rb = np.abs(resized_image[:,:,0]-resized_image[:,:,2])        
        gb = np.abs(resized_image[:,:,1]-resized_image[:,:,2])
        mask = (rg > tol) | (rb > tol) | (gb > tol)
        resized_image[mask]=[0,0,255]
        # hsv = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV) # convert image to HSV color space
        # hsv = np.array(hsv, dtype = np.float64)
        # hsv[:,:,1] = hsv[:,:,1]*1.5 # scale pixel values up for channel 1
        # hsv[:,:,1][hsv[:,:,1]>255]  = 255
        # hsv[:,:,2] = hsv[:,:,2]*1.5 # scale pixel values up for channel 2
        # hsv[:,:,2][hsv[:,:,2]>255]  = 255
        # hsv = np.array(hsv, dtype = np.uint8)
        # resized_image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        h_box, w_box = h, w

        for i in range (2, 35, 5):
            new_h = int((h/50)*i)
            new_w = int((w/50)*i)
            resized_template= cv2.resize(self.template, (new_w,new_h), interpolation= cv2.INTER_LINEAR)


            result = cv2.matchTemplate(resized_image, resized_template, self.method)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
            print("smthng found")
            self.draw_match_boundaries(resized_image,h_box,w_box,found)
        else:
            found = [(0,0)]
            self.draw_match_boundaries(resized_image,h_box,w_box,found)
            print("nothing found")

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
            # cv2.imshow("result", image)
            # cv2.imwrite("aruco_phone1_detected.jpg", image)
        # rate = rospy.Rate(15)
        pub = rospy.Publisher('aruco_detect', Image, queue_size= 10)
        bridge = CvBridge()
        ros_image = bridge.cv2_to_imgmsg(image, encoding='bgr8')
        pub.publish(ros_image)
        print("draw match boundaries is bounding")
        # rate.sleep()
    def resize_image(self, image, h, w):
        """
        This function resizes an image to a given height and width.
        :param image: The cv2 image we are resizing
        :param h: The integer height of the new image
        :param w: The integer width of the new image
        """  
        print("recieved in resized")  
        h_image, w_image, c = image.shape
        if h_image > h: 
        #sizing down 
            print("shit 1 working")
            down_points = (w, h)
            image_resized = cv2.resize(image, down_points, interpolation= cv2.INTER_LINEAR)
            h_new, w_new = h_image, w_image
        else: 
        #sizing up 
            up_points = (w, h)
            print("shit 2 working", w, h)
            image_resized= cv2.resize(image, up_points, interpolation = cv2.INTER_LINEAR)
            print("it resized")
            print(image_resized.shape)
            h_new, w_new = h, w
        return h_new, w_new, image_resized
        #want to:
        #resize image up 
        #max: furthest image will be (smallest it wil be)
        #minimum: within 2 meters 
    def make_mask (self, image, h, w):
        #this function takes image and tempalte dimensions, checks if theyre equal, resizes if they're not equal, the it masks the templ
        
        h_template, w_template, c_template = im.template
        h_image, w_image, c_image = im.image
        if (h_template == h_image & w_image == w_template):
            w, h = self.template.shape[:-1]
            templateGray = cv2.cvtColor(self.template, cv2.COLOR_BGR2GRAY)
            ret, mask = cv2.threshold(templateGray, 200, 255, cv2.THRESH_BINARY)
            mask_inv = cv2.bitwise_not(mask)
            mask_inv = cv2.cvtColor(mask_inv,cv2.COLOR_GRAY2RGB)
            method = cv2.TM_SQDIFF 
            result = cv2.matchTemplate(image, self.template, method, None, mask=mask_inv) 
        else:
            image_resized=self.resize_image(image, h, w)
            w, h = self.template.shape[:-1]
            templateGray = cv2.cvtColor(self.template, cv2.COLOR_BGR2GRAY)
            ret, mask = cv2.threshold(templateGray, 200, 255, cv2.THRESH_BINARY)
            mask_inv = cv2.bitwise_not(mask)
            mask_inv = cv2.cvtColor(mask_inv,cv2.COLOR_GRAY2RGB)
            method = cv2.TM_SQDIFF 
            result = cv2.matchTemplate(image_resized, self.template, method, None, mask=mask_inv) 
    # print(image.shape, image.dtype)
    # print(template.shape, template.dtype)
    # print(mask_inv.shape, mask_inv.dtype)
    # def load_images(folder):
    #     """
    #     This function gets all the images in the given folder location and returns
    #     them in a list as cv2 images with second list of the corresponding name.
    #     Note there should only be images in this folder.
    #     :param folder: The location of the images.
    #     :return: A list of cv2 images and a list of the corresponding image names.
    #     """
    #     global cv_image
    #     dir_list = os.listdir(folder)
    #     images = []
    #     names = []
    #     for file in dir_list:
    #         image = cv2.imread(folder + '/' + file, cv2.IMREAD_COLOR)
    #         images.append(image)
    #         names.append(file)
    #     return images, names
    def run (self):
        print("HEY")
        h, w = self.template.shape[:2]
        print ("h,w", h,w)
        locs = self.detect_template()
        
        print(locs)
        self.draw_match_boundaries(self.cv_image,locs, h, w)
    
if __name__ == "__main__":
    while not rospy.is_shutdown():
        rospy.init_node('aruco_location')   
        rospy.loginfo("Node 'image_listener' initialized")
        cv2.namedWindow("view")
        ad = aruco_detector()
        # ad.run()
        print("6 works")
        
        rospy.spin()