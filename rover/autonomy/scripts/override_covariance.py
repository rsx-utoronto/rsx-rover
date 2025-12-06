#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from heading_filter import Quaternion
import math

class OverrideCovariance(Node):
    def __init__(self):
        super().__init__('override_covariance')
        # self.imu_sub = rospy.Subscriber('/imu/orient', Imu, self.imu_callback)
        # self.gnss_sub = rospy.Subscriber('/calian_gnss/gps', NavSatFix, self.gnss_callback)
        # self.odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        self.imu_sub = self.create_subscription(Imu, '/imu/orient', self.imu_callback, 10)
        self.gnss_sub = self.create_subscription(NavSatFix, '/calian_gnss/gps', self.gnss_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/rtabmap/odom', self.odom_callback, 10)
        
        # self.imu_pub = rospy.Publisher('/imu/orient/override', Imu, queue_size=1)
        # self.gnss_pub = rospy.Publisher('/calian_gnss/gps/override', NavSatFix, queue_size=1)  
        # self.odom_pub = rospy.Publisher('/rtabmap/odom/override', Odometry, queue_size=1)
        # self.heading_pub = rospy.Publisher("/imu/rpy", Float32, queue_size=10)
        self.imu_pub = self.create_publisher(Imu, '/imu/orient/override', 10)
        self.gnss_pub = self.create_publisher(NavSatFix, '/calian_gnss/gps/override', 10)
        self.odom_pub = self.create_publisher(Odometry, '/rtabmap/odom/override', 10)
        self.heading_pub = self.create_publisher(Float32, "/imu/rpy", 10)
        
        # self.imu_orien_cov_multiplier = rospy.get_param('~imu_orien_cov_multiplier', 1.0)
        # self.imu_ang_vel_cov_multiplier = rospy.get_param('~imu_ang_vel_cov_multiplier', 1.0)
        # self.imu_lin_acc_cov_multiplier = rospy.get_param('~imu_lin_acc_cov_multiplier', 1.0)
        # self.gnss_cov_multiplier = rospy.get_param('~gnss_cov_multiplier', 1.0)
        # self.odom_pose_cov_multiplier = rospy.get_param('~odom_pose_cov_multiplier', 1.0)
        # self.odom_twist_cov_multiplier = rospy.get_param('~odom_twist_cov_multiplier', 1.0)
        self.imu_orien_cov_multiplier = self.declare_parameter('imu_orien_cov_multiplier', 1.0).value
        self.imu_ang_vel_cov_multiplier = self.declare_parameter('imu_ang_vel_cov_multiplier', 1.0).value
        self.imu_lin_acc_cov_multiplier = self.declare_parameter('imu_lin_acc_cov_multiplier', 1.0).value
        self.gnss_cov_multiplier = self.declare_parameter('gnss_cov_multiplier', 1.0).value     
        self.odom_pose_cov_multiplier = self.declare_parameter('odom_pose_cov_multiplier', 1.0).value
        self.odom_twist_cov_multiplier = self.declare_parameter('odom_twist_cov_multiplier', 1.0).value
        print("imu orientation covariance multiplier: ", self.imu_orien_cov_multiplier)
        print("imu angular velocity covariance multiplier: ", self.imu_ang_vel_cov_multiplier)
        print("imu linear acceleration covariance multiplier: ", self.imu_lin_acc_cov_multiplier)
        print("gnss covariance multiplier: ", self.gnss_cov_multiplier)
        print("odom pose covariance multiplier: ", self.odom_pose_cov_multiplier)
        print("odom twist covariance multiplier: ", self.odom_twist_cov_multiplier)

    def imu_callback(self, data: Imu):
        # Override the covariance of the IMU message
        ori = [0] * 9
        ang = [0] * 9
        lin = [0] * 9
        for i in range(9):
            ori[i] = data.orientation_covariance[i] * self.imu_orien_cov_multiplier
            ang[i] = data.angular_velocity_covariance[i] * self.imu_ang_vel_cov_multiplier
            lin[i] = data.linear_acceleration_covariance[i] * self.imu_lin_acc_cov_multiplier
        q = Quaternion(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        h = q.quaternion_to_heading(q)
        data.orientation_covariance = tuple(ori)
        data.angular_velocity_covariance = tuple(ang)
        data.linear_acceleration_covariance = tuple(lin)     
        self.imu_pub.publish(data)       
        self.heading_pub.publish(h*180/math.pi)

    def gnss_callback(self, data):
        # Override the covariance of the GNSS message
        pos = [0] * 9
        for i in range(9):
            pos[i] = data.position_covariance[i] * self.gnss_cov_multiplier
        data.position_covariance = tuple(pos)
        data.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.gnss_pub.publish(data)

    def odom_callback(self, data):
        # Override the covariance of the Odometry message
        pose = [0] * 36
        twist = [0] * 36
        for i in range(36):
            pose[i] = data.pose.covariance[i] * self.odom_pose_cov_multiplier
            twist[i] = data.twist.covariance[i] * self.odom_twist_cov_multiplier
        data.pose.covariance = tuple(pose)
        data.twist.covariance = tuple(twist)
        self.odom_pub.publish(data)

if __name__ == '__main__':
    rclpy.init(args=None)
    
    try:
        override_covariance = OverrideCovariance()
        rclpy.spin(override_covariance)
    except rclpy.exceptions.ROSInterruptException:
        pass