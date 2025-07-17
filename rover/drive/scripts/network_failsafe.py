#!/usr/bin/env python3

# import rospy
import rclpy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class NetworkFailsafe (rclpy.node.Node):
    def __init__(self):
        super().__init__('network_failsafe')
        self.status_sub = self.create_subscription(Bool, "/network_status", self.status_callback, 10)
        self.drive_pub = self.create_publisher(Twist, "/drive", 10)
        self.twist = Twist()
    def status_callback(self, msg):
        if msg.data == False:
            self.get_logger().info("KILL ENGAGED")
            # self.twist = Twist()
            self.twist.linear.x = 0
            self.twist.linear.y = 0
            self.twist.linear.z = 0
            self.twist.angular.x = 0
            self.twist.angular.y = 0
            self.twist.angular.z = 0
            self.drive_pub.publish(self.twist)
            # self.rate.sleep()
        else:
            pass

def main():
    rclpy.init()
    nf = NetworkFailsafe()
    rclpy.spin(nf)
    nf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

