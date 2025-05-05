#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class OdomToPose:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/map/ekf', Odometry, self.odom_callback)
        self.pose_pub = rospy.Publisher('/ekf/pose', PoseStamped, queue_size=1)
        self.pose = PoseStamped()

    def odom_callback(self, data):
        self.pose.header = data.header
        self.pose.pose = data.pose.pose
        self.pose_pub.publish(self.pose)

if __name__ == '__main__':
    rospy.init_node('ekf_odom_to_pose')
    OdomToPose()
    rospy.spin()