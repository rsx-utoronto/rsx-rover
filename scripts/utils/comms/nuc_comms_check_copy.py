#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess
import shutil
import platform

class NetworkChecker(Node):
    def __init__(self):
        super().__init__('nuc_comms_check')

        # Parameters (can be overridden at launch)
        self.declare_parameter('base_ip', '192.168.0.123')
        self.declare_parameter('check_period_s', 1.1)   # seconds between checks
        self.declare_parameter('ping_count', 2)        # how many pings to send per check
        self.declare_parameter('ping_timeout_s', 1.0)  # overall subprocess timeout

        self.host = self.get_parameter('base_ip').get_parameter_value().string_value
        self.period = float(self.get_parameter('check_period_s').get_parameter_value().double_value)
        self.ping_count = int(self.get_parameter('ping_count').get_parameter_value().integer_value)
        self.ping_timeout = float(self.get_parameter('ping_timeout_s').get_parameter_value().double_value)

        # Publisher
        self.pub_network = self.create_publisher( Bool, 'network_status', 1)
        
        # Unix-like: -c count (number), -W timeout (per-packet, sometimes seconds),
        # -w is deadline in some implementations - we'll use -c and -W (seconds)
        # Note: macOS ping uses -W differently; this is a reasonable general approach for Linux.
        self._ping_args_template = ['ping', '-c', str(self.ping_count), '-W', str(int(self.ping_timeout)), self.host]

        # Timer-based check
        self.timer = self.create_timer(self.period, self.check_network)
        self.get_logger().info(f"Network checker started. Pinging {self.host} every {self.period}s.")

    def check_network(self):
        # Build command depending on platform
        cmd = self._ping_args_template

        try:
            # Use a subprocess timeout to avoid indefinite blocking
            res = subprocess.run(cmd, capture_output=True, timeout=self.ping_timeout + 1.0)
            connected = (res.returncode == 0)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f"Ping command timed out when pinging {self.host}")
            connected = False
        except Exception as e:
            self.get_logger().error(f"Error running ping: {e}")
            connected = False

        # Publish and log
        self.pub_network.publish(Bool(data=connected))
        if connected:
            self.get_logger().info(f"{self.host} is reachable.")
        else:
            self.get_logger().warn(f"{self.host} is NOT reachable.")

def main(args=None):
    rclpy.init(args=args)
    node = NetworkChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down network checker (KeyboardInterrupt).")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
