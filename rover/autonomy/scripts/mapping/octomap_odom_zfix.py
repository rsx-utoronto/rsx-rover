#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import tf
"""
Doesn't help to fix the problem, check documentation (https://docs.google.com/document/d/1oCBbmjoqrstUPOOrSD8CYpfelnwaZ9cZcK6Ddsa__c4/edit?usp=sharing) for the another way to fix the problem
"""
class Odom2D:
    def __init__(self):

        self.odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        self.odom_pub = rospy.Publisher('/odom_2d', Odometry, queue_size=10)
        self.prev_stamp = rospy.Time.now()
        self.odom_2d = Odometry()

    def odom_callback(self, msg):
        if msg.header.stamp <= self.prev_stamp:
            rospy.logwarn("Odom message is older than the previous message, ignoring it")
            return
        self.odom_2d.header.stamp = msg.header.stamp # Keeping the header the same so that the time stamp is the same
        self.odom_2d.header.frame_id = msg.header.frame_id
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        euler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        # Projecting 3D pose to 2D
        quaternion_2d = tf.transformations.quaternion_from_euler(0, 0, euler[2]) # yaw is euler[2], set pitch and roll to 0
        self.odom_2d.pose.pose.position.x = position.x
        self.odom_2d.pose.pose.position.y = position.y
        self.odom_2d.pose.pose.position.z = 0.0 # 2D # set z to 0
        self.odom_2d.pose.pose.orientation.x = quaternion_2d[0]
        self.odom_2d.pose.pose.orientation.y = quaternion_2d[1]
        self.odom_2d.pose.pose.orientation.z = quaternion_2d[2]
        self.odom_2d.pose.pose.orientation.w = quaternion_2d[3]
        self.odom_2d.child_frame_id = msg.child_frame_id
        self.odom_2d.pose.covariance = msg.pose.covariance

        # Projecting 3D twist to 2D
        self.odom_2d.twist.twist.linear.x = msg.twist.twist.linear.x
        self.odom_2d.twist.twist.linear.y = msg.twist.twist.linear.y
        self.odom_2d.twist.twist.linear.z = 0.0 # 2D # set z velocity to 0
        self.odom_2d.twist.twist.angular.x = 0.0 # Set roll velocity to 0
        self.odom_2d.twist.twist.angular.y = 0.0
        self.odom_2d.twist.twist.angular.z = msg.twist.twist.angular.z
        self.odom_2d.twist.covariance = msg.twist.covariance

        self.odom_pub.publish(self.odom_2d)

if __name__ == '__main__':
    rospy.init_node('odom_2d')
    odom2d = Odom2D()
    rospy.spin()