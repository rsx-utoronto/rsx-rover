#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class PicSaver(Node):
    def __init__(self):
        super().__init__('pic_saver_node')
        self.pic_sub=self.create_subscription( Image, 'geniecam', self.pic_sub_callback, 10)
        self.counter = 0
        self.bridge=CvBridge()  # Create an instance of CvBridge
        self.num_photos = 25 # TODO: change to predetermined number of photos!
        self.save_path = os.path.join(os.path.dirname(__file__), "../genie_calibration_data")
        self.save_path = os.path.abspath(self.save_path)  # Ensure absolute path
        self.save_path = os.path.join(self.save_path, "flat") # change to desired save path
        os.makedirs(self.save_path, exist_ok=True)

    def pic_sub_callback(self, msg):
        if self.counter < self.num_photos:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg)
                filename = os.path.join(self.save_path, f"image_{self.counter:03d}.png")
                cv2.imwrite(filename, cv_image)
                print(f"Saved image {self.counter} to {filename}")
                self.counter += 1
            except Exception as e:
                self.get_logger().error(f"Error saving image: {e}")

def main(args=None):
    rclpy.init(args=args)
    ps = PicSaver()
    rclpy.spin(ps)
    pc.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
   