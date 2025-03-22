#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import os
import random

class ImageSaverNode:
    def __init__(self, output_dir="~/Downloads/captured_images"):
        self.bridge = CvBridge()
        self.image_counter = random.randint(1, 1000000000)
        self.output_dir = os.path.expanduser(output_dir)
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        # ✅ Subscribe to the ZED camera topic
        self.sub = rospy.Subscriber('/zed_node/rgb/image_rect_color', Image, self.callback, queue_size=1)
        self.cv_image = None

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("ZED Image", self.cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr("Error converting image: %s", e)

    def save_image(self):
        if self.cv_image is not None:
            filename = os.path.join(self.output_dir, f"frame_{self.image_counter:06d}.jpg")
            cv2.imwrite(filename, self.cv_image)
            rospy.loginfo(f"Saved image: {filename}")
            self.image_counter = random.randint(1, 1000000000)

def main():
    rospy.init_node("image_saver_node", anonymous=True)
    saver = ImageSaverNode()

    print("Press ENTER to save an image. Type 'q' + ENTER to quit.")

    while not rospy.is_shutdown():
        key = input()
        if key == 'q':
            break
        saver.save_image()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
