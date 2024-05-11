import rospy
import math
import numpy as np
import cv2
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, TransformStamped
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("/camera/color/image_raw_new",Image, queue_size=1)
        self.spot_pub = rospy.Publisher("/beacon_spot_depth", Float32, queue_size=1)
        self.pose_pub = rospy.Publisher("/light_node/rover_state", StateMsg, queue_size=1)

        self.bridge = CvBridge()
        self.info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.get_coordinates)        
        # self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)        
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)        
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.brightest_spot)
        self.first_image = True
        self.count = 0
        self.beacon_detected = False


def brightest_spot(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") # change to bgr8 for a coloured image and mono8 for greyscale
            if self.first_image:
                self.prev_img = cv_image 
                self.curr_img = cv_image
                self.first_image = False
            else:
                self.prev_img = self.curr_img 
                self.curr_img = cv_image
        except CvBridgeError as e:
            print(e)
        
# more code 
    
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))


