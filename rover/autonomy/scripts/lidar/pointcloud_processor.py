#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2


class PointcloudProcessor(Node):

    def __init__(self):
        super().__init__('pointcloud_processor')

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

        self.subscription  # prevent uanused variable warning

        self.filtered_pub = self.create_publisher(
            PointCloud2,
            '/filtered_points',
            qos_profile
        )

        self.ground_removed_pub = self.create_publisher(
            PointCloud2,
            '/ground_removed',
            qos_profile
        )

        self.obstacles_pub = self.create_publisher(
            PointCloud2,
            '/obstacles',
            qos_profile
        )

        self.get_logger().info('pointcloud_processor node initialized successfully')


    def listener_callback(self, msg):
        # Step 1: filtering
        filtered_cloud = self.filterCloud(msg)
        if filtered_cloud is not None:
            self.filtered_pub.publish(filtered_cloud)

        ground_removed_cloud = self.removeGround(msg)
        if ground_removed_cloud is not None:
            self.ground_removed_pub.publish(ground_removed_cloud)

        obstacles_cloud = self.detectObstacles(msg)
        if obstacles_cloud is not None:
            self.obstacles_pub.publish(obstacles_cloud)

        self.get_logger().info('Processed one frame of Lidar Data')


    def filterCloud(self, cloud):
        return cloud

    def removeGround(self, cloud):
        return cloud

    def detectObstacles(self, cloud):
        return cloud

def main(args=None):
    rclpy.init(args=args)

    pointcloud_processor = PointcloudProcessor()

    rclpy.spin(pointcloud_processor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pointcloud_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()