#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("view", cv_image)
        cv2.waitKey(30)
    except Exception as e:
        rospy.logerr("Could not convert from '{}' to 'bgr8'".format(msg.encoding))

def main():
    rospy.init_node("image_listener")
    rospy.loginfo("Node 'image_listener' initialized")
    
    cv2.namedWindow("view")

    rospy.Subscriber("camera/image", Image, image_callback)
    rospy.spin()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()









