#!/usr/bin/python3

# gps and magnetometer heading filter

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import math

class HeadingFilter:
    def __init__(self):
        self.gnss_sub = rospy.Subscriber('/pose', PoseStamped, self.gnss_callback)
        self.imu_sub = rospy.Subscriber('/imu/orient', Imu, self.imu_callback)
        self.heading_pub = rospy.Publisher('/fused_heading', Imu, queue_size=1)
        self.orientation = Quaternion()
        self.imu = Imu()
        self.gnss_fix = False
        self.accuracy_2d = 1000000
        self.accuracy_3d = 1000000
        self.mag_declination = rospy.get_param('~magnetic_declination_radians')
    
    def gnss_callback(self, data):
        self.orientation.set_heading_to_quaternion(self.orientation.quaternion_to_heading(data.pose.orientation) + self.mag_declination)
        self.accuracy_2d = data.pose.position.z
        # print('received gnss heading:', data.heading, data.valid_fix)

    def imu_callback(self, data):
        self.imu = data
        # print('received imu heading:', data.orientation)

    def publish_heading(self):
        # if self.gnss_fix: 
        if self.accuracy_2d < 1:
            self.imu.orientation.x = self.orientation.x
            self.imu.orientation.y = self.orientation.y
            self.imu.orientation.z = self.orientation.z
            self.imu.orientation.w = self.orientation.w
        self.heading_pub.publish(self.imu)

class Quaternion:
    def __init__(self, x=0, y=0, z=0, w=0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    
    def set_heading_to_quaternion(self, heading):
        # Convert heading (in degrees) to radians
        heading_rad = math.radians(heading)
        
        # Calculate quaternion components
        self.w = math.cos(heading_rad / 2)
        self.x = 0
        self.y = 0
        self.z = math.sin(heading_rad / 2)
        
        return self
    
    def quaternion_to_heading(self, data):
        # Convert input quaternion to heading (in radians)
        return math.atan2(2*(data.w*data.z + data.x*data.y), 1 - 2*(data.y**2 + data.z**2))
    
    def set(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
        return self
    
if __name__ == '__main__':
    rospy.init_node('heading_filter')
    rate = rospy.Rate(10)
    heading_filter = HeadingFilter()
    while not rospy.is_shutdown():
        heading_filter.publish_heading()
        rate.sleep()
        # print('published heading:', heading_filter.imu.orientation)