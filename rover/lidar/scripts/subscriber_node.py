#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,     # or BEST_EFFORT
            durability=DurabilityPolicy.VOLATILE,       # or TRANSIENT_LOCAL
            history=HistoryPolicy.KEEP_LAST             # or KEEP_ALL
        )

        self.subscription = self.create_subscription(
            msg_type=PointCloud2,
            topic='/ouster/points',
            callback=self.listener_callback,
            qos_profile=qos_profile),

        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: %s' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()