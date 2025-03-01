#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import os

class ImageSaverNode:
    def __init__(self, output_dir="captured_images"):
        self.bridge = CvBridge()
        self.image_counter = 0
        self.last_save_time = 0  # Track the last save time in seconds
        self.output_dir = output_dir
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        self.sub = rospy.Subscriber("/zed_node/rgb/image_rect_color", Image, self.callback)
        rospy.loginfo("Image saver node started. Saving images to: " + self.output_dir)

    def callback(self, data):
        current_time = rospy.get_time()
        # Only save an image if at least 1 second has passed since the last save
        if current_time - self.last_save_time < 1.0:
            return
        self.last_save_time = current_time

        rospy.loginfo("Received image. Saving...")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("Error converting image: %s", e)
            return
        
        filename = os.path.join(self.output_dir, "frame_" + str(self.image_counter).zfill(6) + ".jpg")
        cv2.imwrite(filename, cv_image)
        rospy.loginfo("Saved image: " + filename)
        self.image_counter += 1

def main():
    rospy.init_node("image_saver_node", anonymous=True)
    saver = ImageSaverNode(output_dir="captured_images")
    rospy.spin()

if __name__ == "__main__":
    main()
