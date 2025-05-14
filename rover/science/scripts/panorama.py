#!/usr/bin/env python3

import stitcher
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
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
    def __init__(self, sub: Subscriber, pub: Publisher):
        self.sub:Subscriber = Subscriber()
        self.pub:Publisher = Publisher()
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
        self.stitcher.stitch(self.sub.imgfiles)
        print('Panorama completed')

def main():
    rospy.init_node('panorama')
    # sub = Subscriber()
    # pub = Publisher()
    pan = Panorama()
    pan.start(10)

if __name__ == '__main__':
    main()