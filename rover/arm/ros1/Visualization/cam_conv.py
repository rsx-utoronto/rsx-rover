#!/usr/bin/env python3
from __future__ import print_function

# import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
# from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# import numpy

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/encoded_image_topic",Image,self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
            # newImg = cv_image
            # # newImage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # # newImage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # encodeParam = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            # result, encImg = cv2.imencode('.jpg', newImg, encodeParam)
            # if result == False:
            #     print('did not encode')
            # print("hello")
            decImg = cv2.imdecode(cv_image, 1)
            # print(decImg)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(decImg, "bgr8"))
        except CvBridgeError as e:
            print(e)

    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "grey"))
    # except CvBridgeError as e:
    #   print(e)

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)