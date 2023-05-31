#!/usr/bin/env python

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


import roslib
roslib.load_manifest('sensor_msgs')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("/camera/infra1/image_rect_raw_new",Image, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/infra1/image_rect_raw",Image,self.brightest_spot)

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

    def brightest_spot(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        image = cv_image
        radius = int(41)
        # load the image and convert it to grayscale
        orig = image.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # perform a naive attempt to find the (x, y) coordinates of
        # the area of the image with the largest intensity value
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)
        cv2.circle(image, maxLoc, radius, (255, 0, 0), 2)
        # display the results of the naive attempt
        cv2.imshow("Naive", image)
        # apply a Gaussian blur to the image then find the brightest
        # region
        gray = cv2.GaussianBlur(gray, (radius, radius), 0)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)
        image = orig.copy()
        cv2.circle(image, maxLoc, radius, (255, 0, 0), 2)
        # display the results of our newly improved method
        cv2.imshow("Robust", image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
        # endregion
   
    def main(args):
        ic = image_converter()
        rospy.init_node('image_converter', anonymous=True)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()
   
if __name__ == '__main__':
    image_converter.main(sys.argv)
    image_converter.brightest_spot()