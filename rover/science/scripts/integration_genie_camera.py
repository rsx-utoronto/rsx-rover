#! /usr/bin/env python3

import rospy 
from sensor_msgs.msg import Image 
import cv2 
from cv_bridge import CvBridge 
import math
import numpy as np
 
# multicast and action commands are supported
#1. capture image from camera 
#2. convert to cv file
#3. convert to ros done
#4. publish to ros done
print("hi")
#testing 123
print("testing")
def initialize_camera():
    camera = Camera()  
    camera.enable_multicast()  # Enable multicast if supported
    return camera

def capture_image(camera):
    # Send action command to trigger image acquisition
    camera.send_action_command("Trigger")  # Replace with actual command name
    # Wait for image data (this might involve a callback or polling)
    image_data = camera.get_image()  # Replace with actual function to get image
    return image_data

def cv_ros_cv():
    rospy.init_node('image_publisher')
    image_pub = rospy.Publisher('/camera/image', Image, queue_size=10)
    bridge = CvBridge()
    camera = initialize_camera()  # Initialize camera
    while not rospy.is_shutdown():
        genie_image = capture_image(camera)  # Capture image
        if genie_image is not None:
            # Convert to OpenCV format
            cv_image = cv2.cvtColor(genie_image, cv2.COLOR_BAYER_BG2BGR)  # Adjust based on the image format
            ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")  # Convert to ROS Image message
            image_pub.publish(ros_image)  # Publish the ROS image
        else:
            rospy.logerr("No image data to publish")

        rospy.sleep(0.1)  # Adjust based on your requirements

if __name__ == '__main__':
    try:
        cv_ros_cv()
    except rospy.ROSInterruptException:
        pass
        
        
        
        
        
        
        