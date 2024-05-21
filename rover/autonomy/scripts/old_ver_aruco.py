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

# we read in the template we are searching the images for
# TEMPLATE = cv2.imread('/home/rsx/rover_ws/src/rsx-rover/rover/autonomy/scripts/Aruco_Frame.png', cv2.IMREAD_COLOR)
# # the algorithm cv2 will use to detect templates
# METHOD = cv2.TM_CCOEFF_NORMED
# # the threshold of similarity between the template and part of an image that
# # qualifies as being a match
# THRESHOLD = 0.6
# # the colour that the bounding boxes for matches to template should be
# # note that the colour is set to (0,255,0) that is the colour green
# COLOUR = (0,255,0)
# # where we get out images to analyze
# FOLDER = "images" 
#global cv_image

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
        self.template_pub.publish(self.ros_image)

    def image_callback(self, msg):
        print("image_callback is working")
        bridge = CvBridge()
        try:
            self.cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            # print(self.cv_image)
            # cv2.imshow("view", self.cv_image)
            # cv2.waitKey(30)
        except Exception as e:
            rospy.logerr("Could not convert from '{}' to 'bgr8'".format(msg.encoding))
        else: 
            print("going into detect")
            print (self.template.shape)
            self.detect_template(self.cv_image, self.template.shape[0], self.template.shape[1])


#no longer using this: 

# def publish_image():
#     try:
#     #rospy.init_node('image publisher', anonymous=True)
#         print(TEMPLATE)
#         image_pub= rospy.Publisher("/camera/color/image_raw", Image, queue_size=1)
#         image_sub= rospy.Subscriber("/camera/color/image_raw", Image, queue_size=1)
#         bridge = CvBridge()
#         ros_image =bridge.cv2_to_imgmsg(TEMPLATE, encoding='bgr8')
#         image_pub.publish(ros_image)
#         rospy.spin()
#     except CvBridgeError as e:
#         print(e)

    def hsv(self, img, l, u):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([l,128,128]) # setting lower HSV value
        upper = np.array([u,255,255]) # setting upper HSV value
        mask = cv2.inRange(hsv, lower, upper) # generating mask
        return mask

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
        # print("sending to resized")
        # h_new, w_new, resized = self.resize_image(img, h, w)
        # diff = cv2.subtract(self.template, resized)
        # detect_template() missing ly the template match to the patch of the image at
        # said pixel of the same height and width as template, the output is the
        # percentage match
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
            
            # resized_template = cv2.cvtColor(resized_template, cv2.COLOR_BGR2GRAY)
            # gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # path = os.path.dirname(os.path.abspath(__file__))
            # img_path = os.path.join(path, 'aruco_phone2.jpg')
            # img = cv2.imread(img_path, cv2.IMREAD_COLOR)

            # res = np.zeros(resized_image.shape, np.uint8) # creating blank mask for result
            # l = 15 # the lower range of Hue we want
            # u = 30 # the upper range of Hue we want
            # mask = self.hsv(resized_image, l, u)
            # inv_mask = cv2.bitwise_not(mask) # inverting mask
            # gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
            # res1 = cv2.bitwise_and(resized_image, resized_image, mask= mask) # region which has to be in color
            # res2 = cv2.bitwise_and(gray, gray, mask= inv_mask) # region which has to be in grayscale
            # for i in range(3):
            #     res[:, :, i] = res2 # storing grayscale mask to all three slices
            # resized_image = cv2.bitwise_or(res1, res) # joining grayscale and color regions

            # b,g,r = cv2.split(resized_image)
            # b[:] = 0
            # r[:] = 0
            # green_ver = cv2.merge((b,g,r))
            # b_t, g_t, r_t = cv2.split(resized_template)
            # b_t[:] = 0
            # r_t[:] = 0
            # green_ver_t = cv2.merge((b_t, g_t, r_t))
            # result = cv2.matchTemplate(green_ver, green_ver_t, self.method)

            # result = cv2.matchTemplate(inv_ri, resized_template, self.method)
            result = cv2.matchTemplate(resized_image, resized_template, self.method)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
            # print("max_val", max_val)
            if max_val > self.threshold:
                if max_val > max_max_val:
                    max_max_loc = max_loc
                    max_max_val = max_val
                    h_box, w_box = new_h, new_w

        if max_max_val > self.threshold:
            print("max_max_loc", max_max_loc, "max_max_val", max_max_val)
            found = []
            result[max_max_loc[1] - h // 2 : max_max_loc[1] + h // 2 + 1, max_max_loc[0] - w // 2 : max_max_loc[0] + w // 2 + 1] = 0
            found.append(max_max_loc)
            print("smthng found")
            self.draw_match_boundaries(resized_image,h_box,w_box,found)
        else:
            found = [(0,0)]
            self.draw_match_boundaries(resized_image,h_box,w_box,found)
            print("nothing found")

        
        # result = cv2.matchTemplate(resized, self.template, self.method)

#        print("result")
#        print(result)

        # the corners of the template matches found
        # found = []

        # set max val to 1 to start with, so it is above the threshold, and we
        # search for instances of the template above the threshold
        # max_val = 1

        # this function will loop until there are no more potential template matches
        #while max_val > self.threshold:
            # we find the values and the location of the min and max in result
        # min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

            # If the max_val is greater than the threshold, then we do not want some
            # pixel which would be withindetect_template() missing  the bounds of the patch the matched at
            # max_loc to the template to also be labelled as a template. Note that
            # these pixels within are less likely to be the template since they were
            # not the max_loc, thus we set their values in result to 0 so they are
            # below the threshold for a match.
        # if max_val > self.threshold:
        #     # all pixels within the bounds of the patch at max_loc have their
        #     # result set to 0
        #     result[max_loc[1] - h_new // 2:max_loc[1] + h_new // 2 + 1, max_loc[0] - w_new // 2:max_loc[0] + w_new // 2 + 1] = 0
        #     # we found a template match so append the location to found
        #     found.append(max_loc)
        #     print("smthng found")
        #     # we return the locations of all the template matches found
        #     self.draw_match_boundaries(resized,h_new,w_new,found)
        #     #take where confidence is max. of ots abpce thresthhold, return that. take the one that is most confident. 
    
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

    #     h, w = self.template.shape[:2]

    # # get all the images we want to analyze and open as cv2 images
    # # images, names = load_images(FOLDER)

    # # for i in range(len(images)):
    # #     fileName = FOLDER + f"/result_{names[i]}"
    # #     locs = detect_template(TEMPLATE, images[i], h, w)
    # #     draw_match_boundaries(images[i],locs,fileName, h, w)
    # #     print("working")
    
    # locs = detect_template(self.template, self.cv_image, h, w)
    # print(locs)
    # draw_match_boundaries(cv_image,locs, h, w)
    
    #cv2.destroyAllWindows() 