#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np

import getmicroscopeid # local import


class MicroscopeCam:

    def __init__(self):
        camera_name = "GENERAL - UVC : GENERAL - UVC"
        camera_id = getmicroscopeid.get_usb_camera_device(camera_name)
        rate = rospy.Rate(1)

        while (not camera_id) and (not rospy.is_shutdown()):
            print("WARNING: Microscope camera not found.")
            camera_id = getmicroscopeid.get_usb_camera_device(camera_name)
            rate.sleep()
        print(camera_id)
        
        self.rate = rospy.Rate(10)
        self.pub_raw = rospy.Publisher("/microscope", Image, queue_size=10)
        self.pub_compressed = rospy.Publisher("/microscope/compressed", CompressedImage, queue_size=2)

        # Get camera feed
        camera = cv2.VideoCapture(camera_id)

        # Create bridge object
        bridge = CvBridge()

        while not rospy.is_shutdown():

            ret, frame = camera.read()

            if ret: # if frame is read correctly

                # Publish raw image
                img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
                self.pub_raw.publish(img_msg)

                # Publish compressed image
                comp = CompressedImage()
                comp.header.stamp = rospy.Time.now()
                comp.format = "jpeg"
                comp.data = np.array(cv2.imencode(
                    '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50]
                )[1]).tobytes()
                self.pub_compressed.publish(comp)

                self.rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("microscopecam")
        microscopecam = MicroscopeCam()
    except rospy.ROSInterruptException:
        pass
