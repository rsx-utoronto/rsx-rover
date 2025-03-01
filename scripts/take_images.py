#!/usr/bin/env python3

import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import os

class ImageSaver:
    def __init__(self, output_dir="images"):
        self.bridge = CvBridge()
        self.output_dir = output_dir
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

    def save_images(self, img_msgs):
        """
        Convert each ROS image message to an OpenCV image and save it.
        
        Args:
            img_msgs (list): List of ROS image messages.
        """
        for i, img_msg in enumerate(img_msgs):
            try:
                img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            except CvBridgeError as e:
                print("Error converting image message:", e)
                continue

            # Save the image with a zero-padded filename
            filename = os.path.join(self.output_dir, "image_" + str(i).zfill(6) + ".jpg")
            cv.imwrite(filename, img)
            print("Saved image", filename)
        print("Done saving images.")

# Example usage:
# Assuming `received_img_msgs` is a list of ROS image messages:
# saver = ImageSaver(output_dir="my_saved_images")
# saver.save_images(received_img_msgs)
