#!/usr/bin/env python3

import stitcher
import rospy
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
        time.sleep(2)
        if self.save:
            print('Image ' + str(len(self.imgfiles)))
            self.imgfiles.append(data)

    def save1(self):
        self.save = True

    def stop(self):
        self.save = False

    def reset(self):
        self.imgfiles = []
        
# ros publisher
class Publisher:
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

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
        self.sub:Subscriber = sub
        self.pub:Publisher = pub
        self.stitcher = stitcher.Stitcher()

    def start(self, num_images):
        print('Starting panorama')
        self.sub.reset()
        self.sub.save1()
        time.sleep(0.1)
        for i in range(num_images):
            self.sub.stop()
            self.pub.turn(0.1)
            time.sleep(0.01)
            self.pub.stop()
            self.sub.save1()
            time.sleep(1)
        self.sub.stop()
        self.stitcher.stitch(self.sub.imgfiles)
        print('Panorama completed')

def main():
    rospy.init_node('panorama')
    sub = Subscriber()
    pub = Publisher()
    pan = Panorama(sub, pub)
    pan.start(20)

if __name__ == '__main__':
    main()