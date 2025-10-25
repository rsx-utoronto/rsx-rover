#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np


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


    """
    PointCloud2 message:
    Height: 64
    Width:: 1024
    Fields:
    - x
    - y
    - z
    - intensity
    - t
    - reflectivity
    - ring
    - ambient
    - range
    
    """
    def listener_callback(self, msg):
        # self.get_logger().info("--POINTCLOUD RECIEVED--")
        # self.get_logger().info(f"Height: {msg.height}  Width: {msg.width}")
        # self.get_logger().info("FIELDS:")
        # for field in msg.fields:
        #     self.get_logger().info(field.name)

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
        # Only include points in a specific range
        points = np.array([
            [p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8]]
            for p in point_cloud2.read_points(
                cloud, field_names=('x', 'y', 'z', 'intensity', 't', 'reflectivity', 'ring', 'ambient', 'range'), skip_nans=True
            )
        ], dtype=np.float32)

        if points.size == 0:
            self.get_logger().warn("Empty cloud received")

        # Mask to only include points in a certain range
        x_min, x_max = -10.0, 10.0
        y_min, y_max = -10.0, 10.0
        z_min, z_max = -10.0, 10.0

        in_x = (points[:, 0] >= x_min) & (points[:, 0] <= x_max)
        in_y = (points[:, 1] >= y_min) & (points[:, 1] <= y_max)
        in_z = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)

        mask = in_x & in_y & in_z
        filtered_points = points[mask]

        # Mask to exclude points in a certain range
        x_min, x_max = -0.1, 0.1
        y_min, y_max = -0.1, 0.1
        z_min, z_max = -0.1, 0.1

        # in_x = (points[:, 0] <= x_min) | (points[:, 0] >= x_max)
        # in_y = (points[:, 1] <= y_min) | (points[:, 1] >= y_max)
        # in_z = (points[:, 2] <= z_min) | (points[:, 2] >= z_max)

        # mask = in_x & in_y & in_z

        mask = ((points[:, 0]**2 + points[:, 1]**2 + points[:, 2]**2) > 0.5)
        filtered_points = points[mask]


        # Define PointFields using official datatypes (FLOAT32 = 7)

        # Create a new PointCloud2 message using the filtered points
        filtered_msg = point_cloud2.create_cloud(
            header=cloud.header,
            fields=cloud.fields,
            points=filtered_points.tolist(),
        )

        return filtered_msg

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