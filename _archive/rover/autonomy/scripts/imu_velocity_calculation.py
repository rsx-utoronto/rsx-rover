#!/usr/bin/env python3

# TODO
# Code breaks when a rosbag is looped (because the IMU time resets)
# Maybe use vectors instead of repetitive variables for x, y and z?

# Problem:
# Values keep going up (acceleration values are always positive, so it keeps going up)
# Likely due to noise, maybe error drift, maybe accleration due to g?

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np


class LinVel:

    def __init__(self):

        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0

        self.vel_x_prev = 0
        self.vel_y_prev = 0
        self.vel_z_prev = 0
        self.time_prev = 0

        self.acc_arr_x = np.array([])
        self.acc_arr_y = np.array([])
        self.acc_arr_z = np.array([])

        rospy.loginfo("Begin!")

        rospy.Subscriber("/imu", Imu, self.imu_callback)
        pub = rospy.Publisher('/linear_velocity', Odometry, queue_size=10)

        odom = Odometry()

        # Set publishing rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            # Publish Linear Velocity
            odom.twist.twist.linear.x = self.vel_x
            odom.twist.twist.linear.y = self.vel_y
            odom.twist.twist.linear.z = self.vel_z

            # Publish odom message
            pub.publish(odom)

            # Not sure....
            rate.sleep()

        # rospy.spin()

    def imu_callback(self, data):

        # Get acceleration values
        a_x = data.linear_acceleration.x
        a_y = data.linear_acceleration.y
        a_z = data.linear_acceleration.z

        # Corrected z (account for g acceleration)
        # a_z = data.linear_acceleration.z + 9.8

        # Set the time interval for the average
        dt = 1

        # Set initial time (when t_0), it's in epoch time
        if (self.time_prev == 0):
            rospy.loginfo("Start Calculating")
            self.time_prev = data.header.stamp.secs

        # rospy.loginfo(self.acc_arr.size)

        # Check if 1 second has elapsed using time from IMU (Alternatively, use rospy.get_time())
        if (data.header.stamp.secs - self.time_prev == dt):

            # Get average acceleration
            acc_avg_x = np.average(self.acc_arr_x)
            acc_avg_y = np.average(self.acc_arr_y)
            acc_avg_z = np.average(self.acc_arr_z)

            rospy.loginfo(self.vel_x)

            # Calculate linear velocity v2 = v1 + a * t
            self.vel_x = self.vel_x_prev + (acc_avg_x * dt)
            self.vel_y = self.vel_y_prev + (acc_avg_y * dt)
            self.vel_z = self.vel_z_prev + (acc_avg_z * dt)

            # rospy.loginfo(self.vel_x);

            # Update previous velocity and time
            self.vel_x_prev = self.vel_x
            self.vel_y_prev = self.vel_y
            self.vel_z_prev = self.vel_z

            # Unideal way to clear array with acceleration values
            del self.acc_arr_x
            del self.acc_arr_y
            del self.acc_arr_z

            # Create new arrays
            self.acc_arr_x = np.array([])
            self.acc_arr_y = np.array([])
            self.acc_arr_z = np.array([])

            # Update time
            self.time_prev = data.header.stamp.secs

        # Accumulate accleration values to average
        self.acc_arr_x = np.append(self.acc_arr_x, a_x)
        self.acc_arr_y = np.append(self.acc_arr_y, a_y)
        self.acc_arr_z = np.append(self.acc_arr_z, a_z)


if __name__ == '__main__':
    rospy.init_node('linear_velocity')

    linVel = LinVel()
