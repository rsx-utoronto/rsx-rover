#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image  # Import Image message type from sensor_msgs package
from cv_bridge import CvBridge
import io

from PIL import Image as imagepillow
import cv2 

QUALITY = 1
PUBLISH_RATE = 5
INPUT_TOPIC = '/zed_node/rgb/image_rect_color'
OUTPUT_TOPIC = 'c_stream'

class compressedImage():

    def __init__(self):
        self.inStream = rospy.Subscriber(INPUT_TOPIC, Image, self.callback, queue_size = 1)
        self.outStream = rospy.Publisher(OUTPUT_TOPIC, Image, queue_size=1)
        self.bridge = CvBridge()
        self.compressed = Image()

    def callback(self, data):
        raw = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        
        _ , compressedBuffer = cv2.imencode('.jpg', raw, [cv2.IMWRITE_JPEG_QUALITY, QUALITY])  
        decBuffer = cv2.imdecode(compressedBuffer, 1)

        self.compressed = self.bridge.cv2_to_imgmsg(decBuffer, encoding="passthrough")
<<<<<<< HEAD
=======
        self.publish()
>>>>>>> master

    def publish(self):
        self.outStream.publish(self.compressed)

if __name__ == '__main__':
    rospy.init_node('compressed', anonymous=True)
    compressed_image = compressedImage()
    rate = rospy.Rate(PUBLISH_RATE)
<<<<<<< HEAD
    while not rospy.is_shutdown():
        compressed_image.publish()
        rate.sleep()
=======
    rospy.spin()
    while not rospy.is_shutdown():
        compressed_image.publish()
        #rate.sleep()
>>>>>>> master
