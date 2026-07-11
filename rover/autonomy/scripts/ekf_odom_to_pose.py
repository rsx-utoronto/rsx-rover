#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class OdomToPose(Node):
    def __init__(self):
        super().__init__('ekf_odom_to_pose')
        self.odom_sub = self.create_subscription( Odometry, '/map/ekf', self.odom_callback,10)
        self.pose_pub = self.create_publisher(PoseStamped,'/ekf/pose', 1)
        self.pose = PoseStamped()

    def odom_callback(self, data):
        self.pose.header = data.header
        self.pose.pose = data.pose.pose
        self.pose_pub.publish(self.pose)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()