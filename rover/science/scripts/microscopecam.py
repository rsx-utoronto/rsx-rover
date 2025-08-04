#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np

import getmicroscopeid # local import


class MicroscopeCam(Node):
    def __init__(self):
        super().__init__("microscopecam")
        camera_name = "GENERAL - UVC : GENERAL - UVC"
        camera_id = getmicroscopeid.get_usb_camera_device(camera_name)
        

        while (not camera_id) and (rclpy.ok()):
            print("WARNING: Microscope camera not found.")
            camera_id = getmicroscopeid.get_usb_camera_device(camera_name)
            rate.sleep()
        print(camera_id)
        
        
        self.pub_raw = self.create_publisher(Image, "/microscope", 10)
        self.pub_compressed = self.create_publisher(CompressedImage, "/microscope/compressed", 2)
        # Get camera feed
        camera = cv2.VideoCapture(camera_id)

        # Create bridge object
        bridge = CvBridge()

        while rclpy.ok():

            ret, frame = camera.read()

            if ret: # if frame is read correctly

                # Publish raw image
                img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
                self.pub_raw.publish(img_msg)

                # Publish compressed image
                comp = CompressedImage()
                comp.header.stamp = self.get_clock().now().to_msg()
                comp.format = "jpeg"
                comp.data = np.array(cv2.imencode(
                    '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50]
                )[1]).tobytes()
                self.pub_compressed.publish(comp)

                time.sleep(1/10)


if __name__ == "__main__":
    rclpy.init()
    try:
        microscopecam = MicroscopeCam()
    except rclpy.exceptions.ROSInterruptException:
        pass
