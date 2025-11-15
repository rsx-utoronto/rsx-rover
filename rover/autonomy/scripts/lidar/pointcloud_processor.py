#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from scipy.spatial import cKDTree
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
import time


class PointcloudProcessor(Node):

    def __init__(self):
        super().__init__('pointcloud_processor')
        # self.frame_skip = 2  # Process every Nth frame

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.subscription = self.create_subscription(
            msg_type=PointCloud2,
            topic='/ouster/points',
            callback=self.listener_callback,
            qos_profile=qos_profile
        )

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

        # Add frame skipping to reduce processing load
        self.frame_skip = 2  # Process every Nth frame
        self.frame_count = 0
        
        # Add processing time tracking
        self.last_process_time = 0.0

        self.get_logger().info('pointcloud_processor node initialized successfully')

    def listener_callback(self, msg):
        # Skip frames to reduce processing load
        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return
        
        start_time = time.time()

        # Single efficient read of all points
        filtered_cloud = self.filterCloud(msg)
        if filtered_cloud is not None:
            self.filtered_pub.publish(filtered_cloud)

        ground_removed_cloud = self.removeGround(filtered_cloud or msg)
        if ground_removed_cloud is not None:
            self.ground_removed_pub.publish(ground_removed_cloud)

        obstacles_cloud = self.detectObstacles(ground_removed_cloud or filtered_cloud or msg)
        if obstacles_cloud is not None:
            self.obstacles_pub.publish(obstacles_cloud)

        process_time = time.time() - start_time
        self.get_logger().info(f'Processed frame in {process_time:.3f}s')

    def filterCloud(self, cloud):
        # Efficient structured array read
        points_list = np.array([
            [p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8]]
            for p in point_cloud2.read_points(
                cloud, field_names=('x', 'y', 'z', 'intensity', 't', 'reflectivity', 'ring', 'ambient', 'range'), skip_nans=True
            )
        ], dtype=np.float32)
        
        if not points_list.size:
            self.get_logger().warn("Empty cloud received")
            return None
            
        points = np.array(points_list, dtype=np.float32)

        # Include points in range
        x_min, x_max = -10.0, 10.0
        y_min, y_max = -10.0, 10.0
        z_min, z_max = -10.0, 10.0

        mask = ((points[:, 0] >= x_min) & (points[:, 0] <= x_max) &
                (points[:, 1] >= y_min) & (points[:, 1] <= y_max) &
                (points[:, 2] >= z_min) & (points[:, 2] <= z_max))
        
        filtered_points = points[mask]

        # Exclude points near origin (robot body)
        x_min, x_max = -0.5, 0.5
        y_min, y_max = -0.5, 0.5
        z_min, z_max = -0.5, 0.5

        mask = ((filtered_points[:, 0] <= x_min) | (filtered_points[:, 0] >= x_max) |
                (filtered_points[:, 1] <= y_min) | (filtered_points[:, 1] >= y_max) |
                (filtered_points[:, 2] <= z_min) | (filtered_points[:, 2] >= z_max))

        filtered_points = filtered_points[mask]

        if filtered_points.size == 0:
            return None

        filtered_msg = point_cloud2.create_cloud(
            header=cloud.header,
            fields=cloud.fields,
            points=filtered_points.tolist(),
        )

        return filtered_msg

    def removeGround(self, cloud, distance_threshold=0.2, max_iterations=50):
        # Reduced iterations for speed
        points_list = np.array([
            [p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8]]
            for p in point_cloud2.read_points(
                cloud, field_names=('x', 'y', 'z', 'intensity', 't', 'reflectivity', 'ring', 'ambient', 'range'), skip_nans=True
            )
        ], dtype=np.float32)

        if not points_list.size:
            self.get_logger().warning("removeGround: empty cloud")
            return None

        points = np.array(points_list, dtype=np.float32)
        xyz = points[:, 0:3]

        n_points = xyz.shape[0]
        if n_points < 3:
            self.get_logger().warning("removeGround: not enough points to fit plane")
            return None

        # Downsample for RANSAC speed (use every Nth point)
        sample_rate = max(1, n_points // 5000)  # Limit to ~5000 points for RANSAC
        xyz_sample = xyz[::sample_rate]
        
        best_inliers = None
        best_count = 0
        best_plane = None

        for _ in range(max_iterations):
            ids = np.random.choice(len(xyz_sample), 3, replace=False)
            p0, p1, p2 = xyz_sample[ids]

            v1 = p1 - p0
            v2 = p2 - p0
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)
            if norm == 0:
                continue
            normal = normal / norm
            d = -np.dot(normal, p0)

            # Test on sampled points first
            dist = np.abs(np.dot(xyz_sample, normal) + d)
            inliers_sample = dist < distance_threshold
            count = np.count_nonzero(inliers_sample)
            
            if count > best_count:
                best_count = count
                best_plane = (normal, d)
                if count > 0.8 * len(xyz_sample):
                    break

        if best_plane is None:
            self.get_logger().info("removeGround: no plane found")
            return cloud

        # Apply best plane to full point cloud
        normal, d = best_plane
        dist = np.abs(np.dot(xyz, normal) + d)
        best_inliers = dist < distance_threshold

        non_ground_points = points[~best_inliers]
        if non_ground_points.size == 0:
            self.get_logger().info("removeGround: all points classified as ground")
            return None

        filtered_msg = point_cloud2.create_cloud(
            header=cloud.header,
            fields=cloud.fields,
            points=non_ground_points.tolist()
        )

        return filtered_msg

    def detectObstacles(self, cloud):
        tolerance = 0.1
        min_cluster_size = 20
        
        points_list = np.array([
            [p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8]]
            for p in point_cloud2.read_points(
                cloud, field_names=('x', 'y', 'z', 'intensity', 't', 'reflectivity', 'ring', 'ambient', 'range'), skip_nans=True
            )
        ], dtype=np.float32)
        
        if not points_list.size:
            self.get_logger().info("detectObstacles: empty cloud")
            return None

        points = np.array(points_list, dtype=np.float32)
        xy = points[:, :2]
        
        # Downsample for clustering if too many points
        if len(xy) > 10000:
            downsample_rate = len(xy) // 10000
            xy_clustered = xy[::downsample_rate]
            points_clustered = points[::downsample_rate]
            idx_map = np.arange(0, len(points), downsample_rate)
        else:
            xy_clustered = xy
            points_clustered = points
            idx_map = np.arange(len(points))
        
        tree = cKDTree(xy_clustered)
        neighbors = tree.query_ball_tree(tree, r=tolerance)
        visited = np.zeros(len(xy_clustered), dtype=bool)
        clusters = []

        for i in range(len(xy_clustered)):
            if visited[i]:
                continue

            stack = [i]
            visited[i] = True
            curr_cluster = []

            while stack:
                curr_index = stack.pop()
                curr_cluster.append(curr_index)

                for j in neighbors[curr_index]:
                    if not visited[j]:
                        visited[j] = True
                        stack.append(j)

            if len(curr_cluster) >= min_cluster_size:
                clusters.append(np.array(curr_cluster, dtype=np.int32))

        if not clusters:
            self.get_logger().info("detectObstacles: no clusters large enough")
            return None

        obstacle_indices = np.concatenate(clusters)
        obstacle_points = points_clustered[obstacle_indices]

        obstacles_msg = point_cloud2.create_cloud(
            header=cloud.header,
            fields=cloud.fields,
            points=obstacle_points.tolist()
        )

        self.get_logger().info(f'detectObstacles: {len(clusters)} clusters, {len(obstacle_points)} points')
        return obstacles_msg


def main(args=None):
    rclpy.init(args=args)
    pointcloud_processor = PointcloudProcessor()
    rclpy.spin(pointcloud_processor)
    pointcloud_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()