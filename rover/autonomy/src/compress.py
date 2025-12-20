#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # Import Image message type from sensor_msgs package
from cv_bridge import CvBridge
import io

from PIL import Image as imagepillow
import cv2 

QUALITY = 1
PUBLISH_RATE = 5
INPUT_TOPIC = '/zed_node/rgb/image_rect_color'
OUTPUT_TOPIC = 'c_stream'


class compressedImage(Node):

    def __init__(self):
        super().__init__('compressed')
        self.inStream = self.create_subscription(Image, INPUT_TOPIC, self.callback,  1)
        self.outStream = self.create_publisher(Image, OUTPUT_TOPIC, 1)
        self.bridge = CvBridge()
        self.compressed = Image()

    def callback(self, data):
        raw = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        
        _ , compressedBuffer = cv2.imencode('.jpg', raw, [cv2.IMWRITE_JPEG_QUALITY, QUALITY])  
        decBuffer = cv2.imdecode(compressedBuffer, 1)

        self.compressed = self.bridge.cv2_to_imgmsg(decBuffer, encoding="passthrough")
        self.outStream.publish(self.compressed)


def main(args=None):
    rclpy.init(args=args)
    compressed_image = compressedImage()
    rclpy.spin(compressed_image)
    compressed_image.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()