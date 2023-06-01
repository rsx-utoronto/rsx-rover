#!/usr/bin/env python
import roslib
roslib.load_manifest('sensor_msgs')
import sys
import time
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("/camera/infra1/image_rect_raw_new",Image, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/infra1/image_rect_raw",Image,self.brightest_spot)

        self.info_sub = rospy.Subscriber("/camera/infra1/camera_info", CameraInfo ,self.get_coordinates)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50,50), 10, 255)
   
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
   
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def thresholding(self, img):
        ret,th1 = cv2.threshold(img,240,255,cv2.THRESH_BINARY)
        # th2 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,11,2)
        # th3 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)
        # titles = ['Original Image', 'Global Thresholding (v = 127)', 'Adaptive Mean Thresholding', 'Adaptive Gaussian Thresholding']
        # images = [img, th1, th2, th3]
        return th1
    
    def colour_search_and_masking(self, img, lower_bound, upper_bound):
        mask = cv2.inRange(img, lower_bound, upper_bound)
        
        detected_output = cv2.bitwise_and(img, img, mask =  mask) 
        # cv2.imshow("red color detection", detected_output)
        return detected_output
    
    def brightest_spot(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") # change to bgr8 for a coloured image and mono8 for greyscale
        except CvBridgeError as e:
            print(e)
        
        image = cv_image

        radius = int(41)
        orig = image.copy()
        # filtered_image = self.thresholding(image)
        filtered_image = self.colour_search_and_masking(image, (185, 185, 185), (195, 195, 195))
        res = image.copy()
        res = cv2.undistort(image, K, D)
        # image = cv2.rotate(image, cv2.ROTATE_180)
        print(res)
        
        # load the image and convert it to grayscale
        
        gray = cv2.cvtColor(filtered_image, cv2.COLOR_BGR2GRAY)  # For a coloured image
        # perform a naive attempt to find the (x, y) coordinates of
        # the area of the image with the largest intensity value
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)
        cv2.circle(image, maxLoc, radius, (255, 0, 0), 2) # (0, 191, 255) is the code for amber colour
        # display the results of the naive attempt
        cv2.imshow("Amber Spot", image)
        cv2.imshow("Original", orig)
        cv2.imshow("Filtered", filtered_image)



        """
        # apply a Gaussian blur to the image then find the brightest
        # region
        gray = cv2.GaussianBlur(gray, (radius, radius), 0)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)
        image = orig.copy()
        cv2.circle(image, maxLoc, radius, (255, 0, 0), 2)
        # display the results of our newly improved method
        cv2.imshow("Robust", image) """
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
        
        
        """# endregion"""
   
    def get_coordinates(self, data):

        # print(data)
        global K
        K = [[0, 0, 0], [0, 0, 0], [0,0,0]]
        K_raw = data.K
        i = 0
        while i < 3:
            for j in range(3):
                K[i][j] = K_raw[i]
            i+=1 

        K = np.asarray(K)  
        # print(K)
        global D
        D = np.asarray(data.D)
        # print(D)
        # img = cv2.resize(img, (img.shape[1]//3, img.shape[0]//3))
        """ self.glob_img = cv2.undistort(self.glob_img, K, D)
        self.glob_img = cv2.rotate(self.glob_img, cv2.ROTATE_180)

        cv2.imshow("Test", self.glob_img) """



    def main(args):
        ic = image_converter()
        rospy.init_node('image_converter', anonymous=True)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()
   
if __name__ == '__main__':
    while not rospy.is_shutdown():
        image_converter.main(sys.argv)
        # image_converter.get_coordinates()
        # image_converter.brightest_spot()















""" import rospy
from nav_msgs.msg import Image
from cv_bridge import CvBridge
bridge = CvBridge()

sub = rospy.Subscriber("/camera/infra1/image_rect_raw", Image, newOdom)

cv_image = bridge.imgmsg_to_cv2("/camera/infra1/image_rect_raw", desired_encoding='passthrough') """


""" # import the necessary packages
import numpy as np
import argparse
import cv2
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help = "path to the image file")
ap.add_argument("-r", "--radius", type = int,
	help = "radius of Gaussian blur; must be odd")
args = vars(ap.parse_args())
# load the image and convert it to grayscale
image = cv2.imread(args["image"])
orig = image.copy()
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# perform a naive attempt to find the (x, y) coordinates of
# the area of the image with the largest intensity value
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)
cv2.circle(image, maxLoc, 5, (255, 0, 0), 2)
# display the results of the naive attempt
cv2.imshow("Naive", image)
# apply a Gaussian blur to the image then find the brightest
# region
gray = cv2.GaussianBlur(gray, (args["radius"], args["radius"]), 0)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)
image = orig.copy()
cv2.circle(image, maxLoc, args["radius"], (255, 0, 0), 2)
# display the results of our newly improved method
cv2.imshow("Robust", image)
cv2.waitKey(0)

# endregion """

""" 

K = np.asarray([[334.94030171, 0, 280.0627713], 
                [0, 595.99313333, 245.316628], 
                [0, 0, 1]])

D = np.asarray([-0.36600591, 0.20973317, -0.00181088, 0.0010202208, -0.07504754])

def repub_images():
    rospy.init_node("imx219_pub")
    image_pub= rospy.Publisher("imx219_image", Image, queue_size=10)
    bridge=CvBridge()
    rate=rospy.Rate(10)

    cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)540, height=(int)540,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink")
    if not cap.isOpened():
        print("Error")
        return
    while not rospy.is_shutdown():
        ret,frame = cap.read()
        if not ret:
            print("Error")
            break
        # frame = cv2.resize(frame, (frame.shape[1]//3, frame.shape[0]//3))
        frame = cv2.undistort(frame, K, D)
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        image_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = rospy.Time.now()
        image_pub.publish(image_msg)


        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

 """