#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu


class ImuVelCalc:
    def __init__(self):
        self.sub_imu = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.vel_x_prev = 0
        self.vel_y.prev = 0
        self.vel_z_prev = 0
        self.time_prev = 0

        rospy.spin()
    def imu_callback(self, data):
        acc = data.linear_acceleration

        acc_x = acc.x
        acc_y = acc.y
        acc_z = acc.z

        dt = data.header.secs - self.time_prev
        vel_x = self.vel_x_prev + acc_x*dt
        vel_y = self.vel_y_prev + acc_y*dt
        vel_z = self.vel_z_prev + acc_z*dt

        self.vel_x_prev = vel_x
        self.vel_y_prev = vel_y
        self.vel_z_prev = vel_z
        
        self.time_prev = data.header.secs


