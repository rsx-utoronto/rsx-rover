#!/usr/bin/env python3

import stitcher
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time

# ros subscriber
class Subscriber():
    def __init__(self, node: Node, crop_left: int = 0, crop_right: int = 0, crop_top: int = 0, crop_bottom: int = 0):
        self.node = node
        self.bridge = CvBridge()
        self.crop_left = max(0, crop_left)
        self.crop_right = max(0, crop_right)
        self.crop_top = max(0, crop_top)
        self.crop_bottom = max(0, crop_bottom)
        # self.sub = rospy.Subscriber('/zed_node/rgb/image_rect_color', Image, self.callback)
        self.sub = self.node.create_subscription(Image, '/zed/zed_node/rgb/image_rect_color', self.callback, 10)
        self.imgfiles = []
        self.save = False
    
    def callback(self, data):
        print('Received image')
        time.sleep(0.7)
        if self.save:
            try:
                image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)
                return

            height, width = image.shape[:2]
            left = min(self.crop_left, width)
            right = min(self.crop_right, max(0, width - left))
            top = min(self.crop_top, height)
            bottom = min(self.crop_bottom, max(0, height - top))

            end_x = width - right if right > 0 else width
            end_y = height - bottom if bottom > 0 else height

            if left >= end_x or top >= end_y:
                self.node.get_logger().warn(
                    "Panorama crop parameters remove the full frame; saving the original image instead."
                )
                cropped_image = image
            else:
                cropped_image = image[top:end_y, left:end_x]

            print('Image ' + str(len(self.imgfiles)))
            self.imgfiles.append(self.bridge.cv2_to_imgmsg(cropped_image, "bgr8"))
            #cv2.imshow('image', CvBridge().imgmsg_to_cv2(data, "bgr8"))

    def save1(self):
        self.save = True

    def stop(self):
        self.save = False

    def reset(self):
        self.imgfiles = []
        
# ros publisher
class Publisher:
    def __init__(self, node: Node):
        # self.pub = rospy.Publisher('/drive', Twist, queue_size=100)
        self.pub = node.create_publisher(Twist, '/drive', 10)

    def turn(self, angular_vel):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = angular_vel
        self.pub.publish(twist)

    def stop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.pub.publish(twist)


class Panorama(Node):
    def __init__(self):
        super().__init__("panorama")

        self.declare_parameter("crop_left", 0)
        self.declare_parameter("crop_right", 0)
        self.declare_parameter("crop_top", 0)
        self.declare_parameter("crop_bottom", 120)

        crop_left = self.get_parameter("crop_left").get_parameter_value().integer_value
        crop_right = self.get_parameter("crop_right").get_parameter_value().integer_value
        crop_top = self.get_parameter("crop_top").get_parameter_value().integer_value
        crop_bottom = self.get_parameter("crop_bottom").get_parameter_value().integer_value

        self.sub:Subscriber = Subscriber(self, crop_left, crop_right, crop_top, crop_bottom)
        self.pub:Publisher = Publisher(self)
        # self.receive_control = rospy.Subscriber("/pano_control", Bool, self.callback)
        # self.pano_img = rospy.Publisher('/pano_img', Image, queue_size=100)
        # self.result_pub = rospy.Publisher("/pano_result", Image, queue_size=100)
        self.receive_control = self.create_subscription(Bool, "/pano_control", self.callback, 10)
        self.pano_img = self.create_publisher(Image, '/pano_img', 10)
        self.result_pub = self.create_publisher(Image, "/pano_result", 10)
        self.stitcher = stitcher.Stitcher()

    def start(self, num_images):
        print('Starting panorama')
        self.sub.reset()
        self.sub.save1()
        time.sleep(0.1)
        for i in range(num_images):
            #stop saving photos
            self.sub.stop()
            #set our angular velocity and for how long it stays at that angular velocity
            self.pub.turn(0.5)
            time.sleep(0.5)
            #stop the rotating, set angular velocity to zero
            self.pub.stop()
            #save the photo that we stopped on
            self.sub.save1()
            time.sleep(1.0)
        #exit out of loop by not saving photos
        self.sub.stop()
        
        #pass stitcher class the list of images to stitch together
        res = self.stitcher.stitch(self.sub.imgfiles)
        if res is None:
            print("Stitching failed")
            return
        else:
            print("Stitching succeeded")
            
            stitched_image = res
            
            # Convert OpenCV image to ROS Image message
            bridge = CvBridge()
            try:
                # Convert the stitched image to a ROS Image message
                ros_image = bridge.cv2_to_imgmsg(stitched_image, "bgr8")
                # Publish the ROS Image message
                self.result_pub.publish(ros_image)

                # Publish the panorama images as ROS Images
                for img in self.sub.imgfiles:
                    ros_image = bridge.cv2_to_imgmsg(img, "bgr8")
                    self.pano_img.publish(ros_image)

            except CvBridgeError as e:
                print(e)
                
        print('Panorama completed')
    
    def callback(self, data):
        if data.data:
            print('Received control')
            self.start(25)

def main():
    rclpy.init(args=None)
    # sub = Subscriber()
    # pub = Publisher()
    pan = Panorama()
    rclpy.spin(pan)

if __name__ == '__main__':
    main()