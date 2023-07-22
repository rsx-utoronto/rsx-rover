#!/usr/bin/env python3
import roslib
roslib.load_manifest('sensor_msgs')
import sys
import time
import rospy
import cv2
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tf2_ros


class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("/camera/color/image_raw_new",Image, queue_size=1)
        self.spot_pub = rospy.Publisher("/beacon_spot_depth", Float32, queue_size=1)
        self.pose_pub = rospy.Publisher("/beacon_pose", Odometry, queue_size=1)

        self.bridge = CvBridge()
        self.info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.get_coordinates)        
        # self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)        
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)        
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.brightest_spot)
        self.first_image = True
        self.count = 0



    # Not being used for now
    def depth_callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)
        self.depth_image = cv_image

    
    def thresholding(self, img):
        ret,th1 = cv2.threshold(img,240,255,cv2.THRESH_BINARY)
        # th2 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,11,2)
        # th3 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)
        # titles = ['Original Image', 'Global Thresholding (v = 127)', 'Adaptive Mean Thresholding', 'Adaptive Gaussian Thresholding']
        # images = [img, th1, th2, th3]
        return th1
    
    # This method filters out the amber areas in the image to make it easier to detect the amber spot
    def colour_search_and_masking(self, img, lower_bound, upper_bound):
        mask = cv2.inRange(img, lower_bound, upper_bound)
        
        detected_output = cv2.bitwise_and(img, img, mask =  mask) 
        # cv2.imshow("red color detection", detected_output)
        return detected_output
    
    def tf_rotation(self, alpha):
        rot_x = np.array([[1, 0, 0], [0, np.cos((np.pi)/2), -np.sin((np.pi)/2)], [0, np.sin((np.pi)/2), np.cos((np.pi)/2)]])
        rot_z = np.array([[np.cos((np.pi)/2), -np.sin((np.pi)/2), 0], [np.sin((np.pi)/2), np.cos((np.pi)/2), 0], [0, 0, 1]]) 
        beta = rot_x.dot(alpha)
        beta = rot_z.dot(beta)
        return beta
    def tf_baselink_to_odom(self):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        try:
            trans = tfBuffer.lookup_transform('odom', 'base_link', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None
        return trans


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
        
        image = self.curr_img
        radius = int(41)
        orig = self.curr_img.copy()
        diff = cv2.absdiff(self.curr_img, self.prev_img)
        filtered_image = diff # self.thresholding(diff)
        # print("filtered_image = ", filtered_image)
        lower_bound = 253
        upper_bound = 254
        # filtered_image = self.colour_search_and_masking(image, (0, 155, 225), (0, 195, 255))
        # filtered_image = self.colour_search_and_masking(image, (lower_bound, lower_bound, lower_bound), (upper_bound, upper_bound, upper_bound)) # amber colour in greyscale is (189, 189, 189) 
        
        
        ## load the image and convert it to grayscale
        
        gray = cv2.cvtColor(filtered_image, cv2.COLOR_RGB2GRAY)  # For a coloured image
        # filtered_image = self.colour_search_and_masking(gray, (lower_bound, lower_bound, lower_bound), (upper_bound, upper_bound, upper_bound)) # amber colour in greyscale is (189, 189, 189) 

        # perform a naive attempt to find the (x, y) coordinates of
        # the area of the image with the largest intensity value
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)

        # Transforming the maxLoc from the pixel coordinates to the base_link frame
        maxLoc_camera = (np.linalg.inv(self.K)).dot(np.array([maxLoc[0], maxLoc[1], 1]))
        maxLoc_base_link = self.tf_rotation(maxLoc_camera)
        x = 0.1
        y = 0
        z = 0.1
        set_dist = 5
        maxLoc_base_link = np.array([x, y, z]) + maxLoc_base_link
        if self.tf_baselink_to_odom() is not None:
            trans = self.tf_baselink_to_odom()
            odom_x = trans.transform.translation.x + maxLoc_base_link[0] + set_dist
            odom_y = trans.transform.translation.y + maxLoc_base_link[1] + set_dist
            odom_z = trans.transform.translation.z + maxLoc_base_link[2] + set_dist
        target_pose = Odometry()
        target_pose.pose.pose.position.x = odom_x
        target_pose.pose.pose.position.y = odom_y
        target_pose.pose.pose.position.z = odom_z
        self.pose_pub.publish(target_pose)

        
        print("maxLoc_camera_frame = ", maxLoc_camera)
        print("maxLoc_base_link_frame = ", maxLoc_base_link)
        
        max_y = maxLoc[1]
        max_x = maxLoc[0]
        # print("maxLoc = ", maxLoc)
        # print("undistorted point = ", undistorted_image(maxLoc))
        amber_spot_depth = Float32()
        amber_spot_depth = self.depth_image[max_y, max_x] # This is the depth of the amber spot in mm
        if maxVal > 200:
            self.spot_pub.publish(amber_spot_depth)

            # print("Depth of the light beacon(in mm):", amber_spot_depth)

            cv2.circle(image, maxLoc, radius, (255, 0, 0), 2) # (0, 191, 255) is the colour code for amber colour
            self.count += 1
            print(str(self.count) + ". maxVal = ", maxVal)

        # display the results of the naive attempt
        cv2.imshow("Amber Spot", image)
        cv2.imshow("Original", orig)
        cv2.imshow("Filtered", filtered_image)
        # cv2.imshow("Gray", gray)



        """
        # apply a Gaussian blur to the image then find the brightest
        # region
        gray = cv2.GaussianBlur(gray, (radius, radius), 0)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)
        image = orig.copy()
        cv2.circle(image, maxLoc, radius, (255, 0, 0), 2)
        # display the results of our newly improved method
        cv2.imshow("Robust", image) """
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
        
        
        """# endregion"""
   
    def get_coordinates(self, data):

        # print(data)
        # Converting the subscribed K into the desired format for the undistort function
        self.K = [[0, 0, 0], [0, 0, 0], [0,0,0]]
        K_raw = data.K
        count = 0
        for i in range(3):
            for j in range(3):
                self.K[i][j] = K_raw[count]
                count+=1

        self.K = np.asarray(self.K)  
        # print("K = ", self.K)
        self.D = np.asarray(data.D)
        # print("D = ", self.D)

        # img = cv2.resize(img, (img.shape[1]//3, img.shape[0]//3))
        """ self.glob_img = cv2.undistort(self.glob_img, K, D)
        self.glob_img = cv2.rotate(self.glob_img, cv2.ROTATE_180)

        cv2.imshow("Test", self.glob_img) """



    def main(args):
        ic = image_converter()
        rospy.init_node('amber_spot_finder', anonymous=True)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()
   
if __name__ == '__main__':
    while not rospy.is_shutdown():
        image_converter.main(sys.argv)
        # image_converter.get_coordinates()
        # image_converter.brightest_spot()

