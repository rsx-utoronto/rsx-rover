#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

import getmicroscopeid # local import


camera_name = "FHD Camera: FHD Camera"
camera_id = getmicroscopeid.get_usb_camera_device(camera_name)

if not camera_id:
    print("WARNING: web camera not found.")
else:
    print(camera_id)


class MicroscopeCam:

    def __init__(self):
        self.rate = rospy.Rate(10)
        self.pub = rospy.Publisher("/webcam", Image, queue_size=10)


        # Get camera feed
        camera = cv2.VideoCapture(camera_id)

        # Create bridge object
        bridge = CvBridge()

        while not rospy.is_shutdown():

            ret, frame = camera.read()

            if ret: # if frame is read correctly

                # Convert with ros bridge and publish
                self.pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

                self.rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("microscopecam")
        microscopecam = MicroscopeCam()
    except rospy.ROSInterruptException:
        pass
