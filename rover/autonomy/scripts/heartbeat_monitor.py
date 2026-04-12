#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time

class HeartbeatMonitor(Node):
    """
    Monitors the /heartbeat topic from the base station.
    If heartbeat is lost for more than 2 seconds, sends a stop command to the motors.
    """
    def __init__(self):
        super().__init__('heartbeat_monitor')
        self.last_heartbeat = time.time()
        self.heartbeat_lost = False
        
        # Subscribe to heartbeat from base station
        self.subscription = self.create_subscription(
            Joy,
            '/software/joy',
            self.heartbeat_callback,
            10
        )
        
        # Publisher for motor stop commands
        self.motor_pub = self.create_publisher(Twist, '/drive', 10)
        
        # Check heartbeat status every 500ms
        self.timer = self.create_timer(0.5, self.check_heartbeat)
        
        self.get_logger().info('Heartbeat monitor started')

    def heartbeat_callback(self, msg):
        """Called when a heartbeat message is received"""
        self.last_heartbeat = time.time()
        if self.heartbeat_lost:
            self.get_logger().info('Heartbeat restored!')
            self.heartbeat_lost = False

    def check_heartbeat(self):
        """Check if heartbeat is still alive, stop motors if lost"""
        elapsed = time.time() - self.last_heartbeat
        
        if elapsed > 2.0 and not self.heartbeat_lost:
            self.get_logger().error(f'Heartbeat lost! No message for {elapsed:.1f}s. STOPPING MOTORS!')
            self.heartbeat_lost = True
            self.stop_motors()
        elif elapsed > 2.0:
            # Keep publishing stop command while heartbeat is lost
            self.get_logger().info('Heartbeat lost, publishing stop command')
            self.stop_motors()

    def stop_motors(self):
        """Send zero velocity command to stop all motors"""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        self.motor_pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()