#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image  # Import Image message type from sensor_msgs package
from cv_bridge import CvBridge
import cv2 
import numpy as np

QUALITY = 1
PUBLISH_RATE = 5
INPUT_TOPIC = '/zed_node/rgb/image_rect_color'
OUTPUT_TOPIC = 'object_stream'

class objectsImage():

    def __init__(self):
        self.inStream = rospy.Subscriber(INPUT_TOPIC, Image, self.callback, queue_size = 1)
        self.outStream = rospy.Publisher(OUTPUT_TOPIC, Image, queue_size=1)
        self.bridge = CvBridge()
        self.objects = Image()

    def callback(self, data):
        raw = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

        #blur = cv2.GaussianBlur(raw, (3,3), 0)
        grey = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)
        laplacian = cv2.Laplacian(grey, cv2.CV_64F)
        smaller = cv2.convertScaleAbs(laplacian)
        
        self.objects = self.bridge.cv2_to_imgmsg(smaller, encoding="passthrough")
        self.publish()

    def publish(self):
        self.outStream.publish(self.objects)

if __name__ == '__main__':
    rospy.init_node('object_edges', anonymous=True)
    objects_image = objectsImage()
    rospy.spin()
