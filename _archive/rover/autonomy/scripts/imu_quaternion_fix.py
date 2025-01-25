#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu


class ImuQuaternionFix:
    def __init__(self):
        
        self.sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.pub = rospy.Publisher('/imu/data', Imu, queue_size=1)
        rospy.spin()
        # self.rate = rospy.Rate(1)

    def imu_callback(self, data):
        better_data = Imu()
        better_data = data
        better_data.orientation.w = 1.0
        self.pub.publish(better_data)
        # self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('imu_quaternion_fix')
    ImuQuaternionFix()