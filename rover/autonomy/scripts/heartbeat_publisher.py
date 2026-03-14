import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import time

class HeartbeatPublisher(Node):
    def init(self):
        super().init('heartbeat_publisher')
        self.publisher = self.create_publisher(Empty, '/heartbeat', 10)
        self.timer = self.create_timer(1.0, self.publish_heartbeat)

    def publish_heartbeat(self):
        msg = Empty()
        self.publisher.publish(msg)
        self.get_logger().info('Heartbeat sent')


def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if name == 'main':
    main()