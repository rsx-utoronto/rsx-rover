import math
import numpy as np
import roslib
roslib.load_manifest('sensor_msgs')
import sys
import time
import rospy
import cv2
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Empty, EmptyResponse
from rover.msg import StateMsg

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import tf2_ros

cap=cv2.VideoCapture('/home/ubuntu/Downloads/IMG_9602.mov')
while (cap.isOpened()):
    ret, frame = cap.read()
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & OxFF == ord('q'):
        break
    
    cap.release()
    cv2.destroyAllWindows()
    
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
        self.first_image = True # first image it sees
        self.count = 0
        self.beacon_detected = False



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
    def tf_baselink_to_map(self):
        tfBuffer = tf2_ros.Buffer(rospy.Duration(1200.0))  # tf buffer length 
        listener = tf2_ros.TransformListener(tfBuffer)
        try:
            # tfBuffer.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(4))
            trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0))
            return trans
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Could not find the transform between base_link and map")
            return None
        

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
        diff = cv2.absdiff(self.curr_img, self.prev_img) #subtract images 
        filtered_image = diff # self.thresholding(diff)
        # print("filtered_image = ", filtered_image)
        lower_bound = 253
        upper_bound = 254
        # filtered_image = self.colour_search_and_masking(image, (0, 155, 225), (0, 195, 255))
        # filtered_image = self.colour_search_and_masking(image, (lower_bound, lower_bound, lower_bound), (upper_bound, upper_bound, upper_bound)) # amber colour in greyscale is (189, 189, 189) 
        
        
        ## load the image and convert it to grayscale
        
        #gray = cv2.cvtColor(filtered_image, cv2.COLOR_RGB2GRAY)  # For a coloured image
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
        if self.tf_baselink_to_map() is not None:
            rospy.loginfo("map tf")
            trans = self.tf_baselink_to_map()
            odom_x = trans.transform.translation.x + maxLoc_base_link[0] + set_dist
            odom_y = trans.transform.translation.y + maxLoc_base_link[1] + set_dist
            odom_z = trans.transform.translation.z + maxLoc_base_link[2] + set_dist
            target_pose = StateMsg()
            target_pose.light_beacon_goal.pose.pose.position.x = odom_x
            target_pose.light_beacon_goal.pose.pose.position.y = odom_y
            target_pose.light_beacon_goal.pose.pose.position.z = odom_z
            self.pose_pub.publish(target_pose)

        
        # print("maxLoc_camera_frame = ", maxLoc_camera)
        # print("maxLoc_base_link_frame = ", maxLoc_base_link)
        
        max_y = maxLoc[1]
        max_x = maxLoc[0]
        # print("maxLoc = ", maxLoc)
        # print("undistorted point = ", undistorted_image(maxLoc))
        amber_spot_depth = Float32()
        amber_spot_depth = self.depth_image[max_y, max_x] # This is the depth of the amber spot in mm
        self.maxVal = maxVal
        if maxVal > 200:
            self.beacon_detected = True
            self.spot_pub.publish(amber_spot_depth)

            # print("Depth of the light beacon(in mm):", amber_spot_depth)

            cv2.circle(image, maxLoc, radius, (255, 0, 0), 2) # (0, 191, 255) is the colour code for amber colour
            self.count += 1
            print(str(self.count) + ". maxVal = ", maxVal)
        else:
            self.beacon_detected = False

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
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()


class CircleService():
    def __init__(self):
        rospy.init_node('amber_spot_finder', anonymous=True)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.detect_pub = rospy.Publisher("/light_node/rover_state", StateMsg, queue_size=1)
        self.rate = rospy.Rate(1)
        self.odom_sub = rospy.Subscriber("/aft_mapped_to_init", Odometry, self.odom_callback)
        rospy.loginfo("Service /light_beacon_rotate Ready")
        self.my_service = rospy.Service('/light_beacon_rotate', Empty, self.callback)
        # ic = image_converter()

        rospy.spin() # maintain the service open.

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # rospy.loginfo("Odom Callback")
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)   

    def callback(self, request):
        rospy.loginfo("The rotation service has been called")
        init_yaw = self.yaw

        move = Twist()
        move.linear.x = 0.0
        move.angular.z = 0.0
        self.vel_pub.publish(move)

        while abs(init_yaw - self.yaw) < 3.14: # full rotation
            while abs(init_yaw - self.yaw) < 0.524: # 30 degrees
                move.linear.x = 0.0
                move.angular.z = 0.2
                self.vel_pub.publish(move)
                self.rate.sleep()
                cur_yaw = self.yaw
                print("angle moved = ", abs(cur_yaw - self.yaw))
            
            move.angular.z = 0.0
            self.vel_pub.publish(move)
            rospy.loginfo("The rotation has paused")
            time.sleep(5)
            ic = image_converter()
            # ic.main(sys.argv)
            rospy.loginfo("Beacon detection has started/resumed")
            i = 0
            while i < 5:
                if ic.beacon_detected:
                    time.sleep(2)
                    i+=1
                    rospy.loginfo("Checking Again...")
                else:
                    break
            if i ==5:
                rospy.loginfo("Beacon detected")
                self.detect_pub.LIGHT_BEACON_DETECTED.publish(True)
                break

        return EmptyResponse()

        
   
if __name__ == '__main__':
    while not rospy.is_shutdown():
        # rospy.init_node('amber_spot_finder')
        CircleService()
        # image_converter.main(sys.argv)
