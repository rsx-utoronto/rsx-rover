#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import os

class ImageSaverNode:
    def __init__(self, output_dir="~/Downloads/captured_images"):
        self.bridge = CvBridge()
        self.image_counter = 0
        self.output_dir = os.path.expanduser(output_dir)
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        self.sub = rospy.Subscriber("/zed_node/rgb/image_rect_color", Image, self.callback)
        rospy.loginfo("Image saver node started. Saving images to: " + self.output_dir)

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("Error converting image: %s", e)

    def save_image(self):
        if self.cv_image is not None:
            filename = os.path.join(self.output_dir, f"frame_{self.image_counter:06d}.jpg")
            cv2.imwrite(filename, self.cv_image)
            rospy.loginfo(f"Saved image: {filename}")
            self.image_counter += 1
        else:
            rospy.logwarn("No image received yet.")

def main():
    rospy.init_node("image_saver_node", anonymous=True)
    saver = ImageSaverNode(output_dir="~/Downloads/captured_images")

    while not rospy.is_shutdown():
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):  # Spacebar key
            saver.save_image()

if __name__ == "__main__":
    main()
