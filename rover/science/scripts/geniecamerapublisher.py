#!/usr/bin/env python3


"""
Filename: geniecamerapublisher.py

Get camera feed in OpenCV of the GigE-V Genie Camera, and publish it to a ROS Topic.

Credit: A large portion of this code is modified from JSeam2/pyGigE-V on GitHub.
https://github.com/JSeam2/pyGigE-V/blob/master/test.py

Robotics for Space Exploration - University of Toronto

Author: Jason Li <jasonli.li@mail.utoronto.ca>
Date: 2025-01-04
"""





import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge
from pygigev import PyGigEV as gev # pip install pygigev
import os
import time


"""
This code is experiencing performance issues

See the note at the top of the README for explanation:
https://github.com/JSeam2/pyGigE-V
"""




class GenieCameraPublisher:





    def __init__(self):
        """

        Initialize all camera values
        -----------------------------------------

        """


        # create new context to store native camera data
        self.ctx = gev()

        # print list of available cameras
        print(self.ctx.GevGetCameraList())

        # open the first detected camera - returns 'OK'
        self.camera_found = self.ctx.GevOpenCamera() == 'OK'

        # get image parameters - returns python object of params
        params = self.ctx.GevGetImageParameters()
        print("Initial image parameters:")
        print(params)

        # camera sensor properties
        width_max = 1936
        height_max = 1216
        binning = 0
        saturation = 0
        brightness = 0
        contrast = 0

        # desired properties
        crop_factor = 1.0
        self.width = int(width_max * 1/crop_factor)
        self.height = int(height_max * 1/crop_factor)
        x_offset = int((width_max - self.width) / 2)
        y_offset = int((height_max - self.height) / 2)

        self.ctx.GevSetImageParameters(self.width,
                                self.height,
                                x_offset,
                                y_offset,
                                params['pixelFormat'][0])
        params = self.ctx.GevGetImageParameters()
        print("Final image parameters:")
        print(params)

        self.width = params['width']
        self.height = params['height']

        # allocate image buffers and prepare for async image transfer to buffer
        self.ctx.GevInitializeImageTransfer(1)

        # start transfering images to memory buffer,
        # use -1 for streaming or [1-9] for num frames
        self.ctx.GevStartImageTransfer(-1)

        self.take_image_sub = rospy.Subscriber("/save_genie_image", String, self.save_genie_callback)



    def _get_image(self):
        """
        Take the first image in buffer, and return it as a numpy array.
        -----------------------------------------

        returns:
            numpy array
        """

        # simply return numpy array for first image in buffer
        img = self.ctx.GevGetImageBuffer().reshape(self.height, self.width)
        # if self.width > 600 or self.height > 600:
        #    # img = cv2.resize(img, (int(self.width*0.25), int(self.height*0.25)))
        #    img = cv2.resize(img, ( 300,300))

        return img
    


    def capture_image(self, filename="captured_image.jpg"):
        """
        Capture one image from the camera and save it locally.
        """
        img = self._get_image()
        cv2.imwrite(filename, img)
        print(f"Image saved as {filename}")

        

    def publish_ros_topic(self):
        """
        Continuously publish the image to a ROS topic.
        -----------------------------------------
        """

        rospy.init_node("geniecam")

        rate = rospy.Rate(2)
        pub = rospy.Publisher("geniecam", Image, queue_size=10)

        bridge = CvBridge()

        while not rospy.is_shutdown():

            img = self._get_image()
            self.img = bridge.cv2_to_imgmsg(img, "8UC1")
            pub.publish(self.img)

            rate.sleep()

    def save_genie_callback(self, msg):
        msgs = msg.data.split(",")
        try:
            # Create bridge to convert ROS Image to OpenCV image
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(self.img, desired_encoding="passthrough")
            
            # Create directory if it doesn't exist
            images_dir = os.path.join(os.path.expanduser("~"), "rover_ws/src/rsx-rover/science_data", "genie_images", msgs[0])
            if not os.path.exists(images_dir):
                os.makedirs(images_dir)
            
            # Create a timestamp for a unique filename
            timestr = time.strftime("%Y%m%d-%H%M%S")
            filename = os.path.join(images_dir, f"genie_image_{msgs[1]}.png")
            
            # Save the image
            cv2.imwrite(filename, cv_image)
            
            # Notify user
            print(f"Genie image saved to {filename}")
            
        except Exception as e:
            print(f"Error saving genie image: {e}")  




if __name__ == "__main__":
    try:

        g = GenieCameraPublisher()

        while not rospy.is_shutdown() and not g.camera_found:
            print("Waiting for genie camera...")
            rospy.sleep(2)

        if g.camera_found:
            print("Genie camera found!")
            g.publish_ros_topic()
            # g.capture_image("captured_image.jpg")  
        else:
            print("No Genie camera found.")

    except rospy.ROSInterruptException:
        pass