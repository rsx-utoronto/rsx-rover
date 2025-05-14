#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraStoring:
    def __init__(self):
        self.rate = rospy.Rate(2)  
        self.cam_data = None
        self.image = None
        self.bridge= CvBridge()
        # what is the camera node called?
        self.sub1 = rospy.Subscriber('geniecam', Image, callback = self.img_callback)


    def img_callback(self, cam_data : Image):
        #self.cam_data = cam_data.data
        # converts to openCV 
        try:
            cv_image = self.bridge.imgmsg_to_cv2(cam_data, "8UC1")
            self.image = cv_image
            self.saving()
            
        except Exception as e:
            rospy.logerr(f"error: {e}")

    def saving(self):
        if self.image.all():
            #cv2.imwrite("~/testing/image_name.jpeg", self.image)
            cv2.imwrite("/home/rsx-base/rsx-rover/rover/science/scripts/image_name.jpeg", self.image)
            rospy.loginfo(f"image saved")
        else:
            rospy.loginfo("waiting")

if __name__ == '__main__':
    try:
        rospy.init_node('image_saving', anonymous=True)
        image_saving_node = CameraStoring()  
        rospy.spin()
    except rospy.ROSInterruptException:
        pass