#!/usr/bin/env python3

import stitcher
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time

# ros subscriber
class Subscriber:
    def __init__(self):
        self.sub = rospy.Subscriber('/zed_node/rgb/image_rect_color', Image, self.callback)
        self.imgfiles = []
        self.save = False
    
    def callback(self, data):
        print('Received image')
        time.sleep(0.7)
        if self.save:
            print('Image ' + str(len(self.imgfiles)))
            self.imgfiles.append(data)
            #cv2.imshow('image', CvBridge().imgmsg_to_cv2(data, "bgr8"))

    def save1(self):
        self.save = True

    def stop(self):
        self.save = False

    def reset(self):
        self.imgfiles = []
        
# ros publisher
class Publisher:
    def __init__(self):
        self.pub = rospy.Publisher('/drive', Twist, queue_size=100)

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


class Panorama:
    def __init__(self):
        self.sub:Subscriber = Subscriber()
        self.pub:Publisher = Publisher()
        self.receive_control = rospy.Subscriber("/pano_control", Bool, self.callback)
        self.pano_img = rospy.Publisher('/pano_img', Image, queue_size=100)
        self.result_pub = rospy.Publisher("/pano_result", Image, queue_size=100)
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
            time.sleep(1)
            #stop the rotating, set angular velocity to zero
            self.pub.stop()
            #save the photo that we stopped on
            self.sub.save1()
            time.sleep(1)
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
            self.start(10)

def main():
    rospy.init_node('panorama')
    # sub = Subscriber()
    # pub = Publisher()
    pan = Panorama()
    rospy.spin()

if __name__ == '__main__':
    main()