#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraStoring(Node):
    def __init__(self):
        super().__init__('image_saving')
        
        # self.rate = rospy.Rate(2)  
        self.cam_data = None
        self.image = None
        self.bridge= CvBridge()
        # what is the camera node called?
        # self.sub1 = self.Subscriber('geniecam', Image, callback = self.img_callback)
        self.sub1 = self.create_subscription(Image, 'geniecam', self.img_callback, 10)


    def img_callback(self, cam_data : Image):
        #self.cam_data = cam_data.data
        # converts to openCV 
        try:
            cv_image = self.bridge.imgmsg_to_cv2(cam_data, "8UC1")
            self.image = cv_image
            self.saving()
            
        except Exception as e:
            # self.get.logerr(f"error: {e}")
            self.get_logger().error(f"Error converting image: {e}")

    def saving(self):
        if self.image.all():
            #cv2.imwrite("~/testing/image_name.jpeg", self.image)
            cv2.imwrite("/home/rsx-base/rsx-rover/rover/science/scripts/image_name.jpeg", self.image)
         
            self.get_logger().info("Image saved successfully")
        else:
       
            self.get_logger().info("No image to save, waiting for new image...")

if __name__ == '__main__':
    rclpy.init()
    try:
        image_saving_node = CameraStoring()  
        rclpy.spin(image_saving_node)
    except rospy.exceptions.ROSInterruptException:
        pass