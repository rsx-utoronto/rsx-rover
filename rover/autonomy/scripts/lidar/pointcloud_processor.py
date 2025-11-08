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
            qos_profile=qos_profile
        )

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

        # run ground removal on the filtered cloud (fallback to original msg)
        ground_removed_cloud = self.removeGround(filtered_cloud or msg)
        if ground_removed_cloud is not None:
            self.ground_removed_pub.publish(ground_removed_cloud)

        # detect obstacles on the ground-removed cloud (fallbacks)
        obstacles_cloud = self.detectObstacles(ground_removed_cloud or filtered_cloud or msg)
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
        x_min, x_max = -0.5, 0.5
        y_min, y_max = -0.5, 0.5
        z_min, z_max = -0.5, 0.5

        in_x = (filtered_points[:, 0] <= x_min) | (filtered_points[:, 0] >= x_max)
        in_y = (filtered_points[:, 1] <= y_min) | (filtered_points[:, 1] >= y_max)
        in_z = (filtered_points[:, 2] <= z_min) | (filtered_points[:, 2] >= z_max)

        mask = in_x | in_y | in_z

        # mask = ((points[:, 0]**2 + points[:, 1]**2 + points[:, 2]**2) > 0.5)
        filtered_points = filtered_points[mask]


        # Define PointFields using official datatypes (FLOAT32 = 7)

        # Create a new PointCloud2 message using the filtered points
        filtered_msg = point_cloud2.create_cloud(
            header=cloud.header,
            fields=cloud.fields,
            points=filtered_points.tolist(),
        )

        return filtered_msg

    def removeGround(self, cloud, distance_threshold=0.2, max_iterations=100):
        # Read points into a flat float numpy array (same field order as filterCloud)
        pts = [
            [float(p[0]), float(p[1]), float(p[2]),
             float(p[3]) if p[3] is not None else 0.0,
             float(p[4]) if p[4] is not None else 0.0,
             float(p[5]) if p[5] is not None else 0.0,
             float(p[6]) if p[6] is not None else 0.0,
             float(p[7]) if p[7] is not None else 0.0,
             float(p[8]) if p[8] is not None else 0.0]
            for p in point_cloud2.read_points(
                cloud,
                field_names=('x', 'y', 'z', 'intensity', 't', 'reflectivity', 'ring', 'ambient', 'range'),
                skip_nans=True
            )
        ]

        if len(pts) == 0:
            self.get_logger().warning("removeGround: empty cloud")
            return None

        points = np.array(pts, dtype=np.float32)
        xyz = points[:, 0:3]

        best_inliers = None
        best_count = 0

        n_points = xyz.shape[0]
        if n_points < 3:
            self.get_logger().warning("removeGround: not enough points to fit plane")
            return None

        # RANSAC loop
        for _ in range(max_iterations):
            # sample 3 distinct indices
            ids = np.random.choice(n_points, 3, replace=False)
            p0, p1, p2 = xyz[ids]

            # compute plane normal via cross product
            v1 = p1 - p0
            v2 = p2 - p0
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)
            if norm == 0:
                continue
            normal = normal / norm
            d = -np.dot(normal, p0)

            # distances of all points to plane
            dist = np.abs(np.dot(xyz, normal) + d)

            inliers = dist < distance_threshold
            count = np.count_nonzero(inliers)
            if count > best_count:
                best_count = count
                best_inliers = inliers
                # early exit if very good fit
                if best_count > 0.8 * n_points:
                    break

        if best_inliers is None:
            self.get_logger().info("removeGround: no plane found, returning original cloud")
            return cloud

        # keep points that are NOT ground (outliers)
        non_ground_points = points[~best_inliers]
        if non_ground_points.size == 0:
            self.get_logger().info("removeGround: all points classified as ground")
            return None

        filtered_msg = point_cloud2.create_cloud(
            header=cloud.header,
            fields=cloud.fields,
            points=non_ground_points.tolist()
        )

        self.get_logger().info(f"removeGround: removed {int(best_count)} ground points, kept {non_ground_points.shape[0]}")
        return filtered_msg

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