#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker,MarkerArray
import yaml 
import os

file_path = os.path.join(os.path.dirname(__file__), "sm_config.yaml")

with open(file_path, "r") as f:
    sm_config = yaml.safe_load(f)

class AstarObstacleAvoidance(Node):
    def __init__(self, lin_vel = 0.3, ang_vel= 0.3, goal=[(5,0)]):
        super().__init__('octomap_a_star_planner')
        print("initializing astar")

        self.z_min = 0.1  # Minimum height to consider as an obstacle
        self.z_max= 2.0  # Maximum height to consider as an obstacle
        self.obstacle_threshold = 1  # Value to mark occupied cells in the grid
        self.grid_size = (100, 100)  # Size of the occupancy grid (width, height)
        self.resolution = 0.1  # Meters per grid cell

        ##### NOTEE: ACCESS TO REALSENSE DOES NOT WORK!!!
        self.declare_parameter('realsense_detection', False)
        self.declare_parameter('realsense_pointcloud', '/camera/depth/points') #obstacle avoidance points from realsense
        self.declare_parameter('pointcloud_topic', '/zed_node/point_cloud/cloud_registered') #obsatcle avoidance points from zed


        realsense_enabled = self.get_parameter('realsense_detection').get_parameter_value().bool_value
        realsense_topic = self.get_parameter('realsense_pointcloud').get_parameter_value().string_value
        fallback_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value

        if realsense_enabled: #check which camera we're using
            self.pointcloud_topic = realsense_topic
        else:
            self.pointcloud_topic = fallback_topic

        self.pointcloud_sub = self.create_subscription(PointCloud2, self.pointcloud_topic, self.pointcloud_callback, 10)
        self.invalid_pose_pub = self.create_publisher(Marker, "/invalid_pose_markers", 10)
    
    def pointcloud_callback(self, msg):
        print("in pointcloud callback")
        """
        Take ZED PointCloud2 → update OcTree with both occupied + free voxels → publish OctoMap.
        """
        xyz = np.array([[point[0], point[1], point[2]] for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)]) #getting the points --> from where?
        mask = np.isfinite(xyz).all(axis=1)
        xyz = xyz[mask] #to filtre out non-finite numbers

        w,h = self.grid_size
        new_height_grid = np.zeros((h, w), dtype=np.float32)  # heights
        grid = np.zeros((h, w), dtype=np.int8)   # shape = (rows, cols) #confirm shape!!!

        for x,y,z in xyz: #go through all points
            if not (self.z_min < z < self.z_max):
                print("not in threshold", z)
                continue


            gx,gy=self.world_to_grid(x,y) #transform the loc into our coord
            if 0 <= gx < w and 0 <= gy < h: #whithin the height threshhold so assign obstacle_threshold
                grid[gy, gx] = self.obstacle_threshold   # mark as occupied
                new_height_grid[gy, gx] = z  # Store actual height
                world_x, world_y = self.grid_to_world(gx, gy) #transform back
                self.publish_invalid_pose_marker(world_x, world_y)  # Publish invalid pose marker --> is it beacuse we've assigned its value to grid?
        
        self.height_grid=new_height_grid
        self.occupancy_grid=grid
    
    def publish_invalid_pose_marker(self, x, y, z=0.1):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "invalid_pose"
        marker.id = int(self.get_clock().now().nanoseconds / 1e6) % 1000000  # give it a semi-unique ID
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
       # marker.lifetime = MsgDuration(sec=8, nanosec=0)  # markers disappear after 8 seconds

        self.invalid_pose_pub.publish(marker)

    def navigate(self, start, goal):
        self.start=0
        # Placeholder for A* path planning logic
        

def main(args=None):
    rclpy.init(args=args)
    planner = AstarObstacleAvoidance()
    
    # Spin in a separate thread so callbacks work while navigate() runs
    from threading import Thread
    spin_thread = Thread(target=rclpy.spin, args=(planner,))
    spin_thread.start()
    
    try:
        planner.navigate(0,4)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()       # signals spin() to return
        spin_thread.join()     # wait for the spin thread to exit
        planner.destroy_node()

if __name__ == '__main__':
    main()