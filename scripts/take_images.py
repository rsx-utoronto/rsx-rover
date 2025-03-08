#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import os
import sys
import select

class ImageSaverNode:
    def __init__(self, output_dir="~/Downloads/captured_images"):
        self.bridge = CvBridge()
        self.image_counter = 0
        self.output_dir = os.path.expanduser(output_dir)
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        # Subscribe to ZED topic
        self.sub = rospy.Subscriber('/zed_node/rgb/image_rect_color', Image, self.callback, queue_size=1)
        rospy.loginfo("Subscribed to /zed_node/rgb/image_rect_color")
        self.cv_image = None

    def callback(self, data):
        rospy.loginfo("Received image message!")
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            rospy.loginfo("Image converted successfully!")
            cv2.imshow("ZED Image", self.cv_image)
            cv2.waitKey(1)  # Keep this for display
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
    saver = ImageSaverNode()

    rospy.loginfo("Press SPACE to save an image.")

    # Use select to detect keypresses (works over SSH)
    while not rospy.is_shutdown():
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == ' ':
                saver.save_image()

if __name__ == "__main__":
    main()
