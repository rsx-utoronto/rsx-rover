#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu


class ImuDataToENU:
    def __init__(self):
        
        self.sub = rospy.Subscriber('/zed_node/imu/data', Imu, self.imu_callback)
        self.pub = rospy.Publisher('/imu/enu', Imu, queue_size=1)
        rospy.spin()

    def imu_callback(self, data):
        transformed = Imu()
        transformed.header.stamp = data.header.stamp
        transformed.header.frame_id = 'base_link'
        transformed.linear_acceleration.x = -data.linear_acceleration.y
        transformed.linear_acceleration.y = data.linear_acceleration.x
        transformed.linear_acceleration.z = data.linear_acceleration.z
        transformed.angular_velocity.x = -data.angular_velocity.y
        transformed.angular_velocity.y = data.angular_velocity.x
        transformed.angular_velocity.z = data.angular_velocity.z
        self.pub.publish(transformed)

if __name__ == '__main__':
    rospy.init_node('imu_to_enu')
    ImuDataToENU()