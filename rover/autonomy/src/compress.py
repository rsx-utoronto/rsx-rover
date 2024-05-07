#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image  # Import Image message type from sensor_msgs package
from cv_bridge import CvBridge
import io

from PIL import Image as imagepillow
import cv2 

def callback(data):
    
    # Compressed image quality (0 - 100)    
    quality = 1

    # Publishing rate (should be lower than publishing rate of the image_rect_color topic)
    publishRate = 15
    
    bridge = CvBridge()

    cstream = rospy.Publisher('cstream', Image, queue_size=10)
    rate = rospy.Rate(publishRate) # publish at 5hz        

    # while not rospy.is_shutdown():

    raw = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    
    # Save to disk
    # cv2.imwrite('compress.jpg', raw, [cv2.IMWRITE_JPEG_QUALITY, quality])
    # cv2.imwrite("raw.jpg", raw, [cv2.IMWRITE_JPEG_QUALITY, 100]) 

    # In memory compression (encode and decode)
    _ , compressedBuffer = cv2.imencode('.jpg', raw, [cv2.IMWRITE_JPEG_QUALITY, quality])  
    decBuffer = cv2.imdecode(compressedBuffer, 1)

    compressed_msg = bridge.cv2_to_imgmsg(decBuffer, encoding="passthrough")

    cstream.publish(compressed_msg)

    rate.sleep()

    # Compression comparsion
    
    # rawSize = rawBuffer.size
    # compressedSize = compressedBuffer.size
    # print(f"Raw Size: {rawSize}, Compressed Size: {compressedSize}")
    # print(f"Reduction: {(rawSize - compressedSize) / rawSize * 100}%")    

def subscriber():

    rospy.init_node('compressnode', anonymous=True)

    # Subscribe to the topic "/zed/zed_node/rgb/image_rect_color" with the message type Image
    # Envoke callback
    rospy.Subscriber('/zed/zed_node/rgb/image_rect_color', Image, callback)

    # Keep running node
    rospy.spin()

if __name__ == '__main__':
    subscriber()
