#!/usr/bin/env python3

"""
A* path planning with OctoMap integration for obstacle avoidance.
- Goal: Compute a path to a goal while avoiding obstacles using height and occupancy costs.
- Obstacles below 0.2m height are considered traversable; above are avoided.
- Publishes resulting path as nav_msgs/Path for RViz visualization.
"""

import rospy
import numpy as np
from nav_msgs.msg import Odometry, Path
from octomap_msgs.msg import Octomap
from geometry_msgs.msg import PoseStamped, Twist, Point
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
import ros_numpy
from octree import OctreeNode
from queue import PriorityQueue
from std_msgs.msg import Header
import math
import tf.transformations as tf

class OctoMapAStar:
    def __init__(self):
        rospy.init_node("octomap_a_star_planner", anonymous=True)

        # Parameters
        self.map_topic = rospy.get_param("~map_topic", "/octomap_binary")
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "/zed/point_cloud/cloud_registered")
        self.odom_topic = rospy.get_param("~odom_topic", "/rtabmap/odom")
        self.tree_resolution = rospy.get_param("~resolution", 0.1)
        self.goal = (9, 7)  # Default goal (x, y)
        self.grid_resolution = 0.1
        self.grid_origin = (0.0, 0.0)
        self.update_rate = rospy.Rate(10)

        # Publishers and Subscribers
        self.path_pub = rospy.Publisher("/astar_path", Path, queue_size=10)
        self.map_sub = rospy.Subscriber(self.map_topic, Octomap, self.octomap_callback)
        self.pointcloud_sub = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.pointcloud_callback)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        
        self.tree = OctreeNode(((-100, -100, -5), (100, 100, 5)), self.tree_resolution)
        self.occupancy_grid = None

        self.current_position = (0, 0)

    def odom_callback(self, msg):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def pointcloud_callback(self, msg):
        print("in pointcloud _callback")
        point_cloud = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
        points = np.zeros((point_cloud.shape[0], 3), dtype=np.float32)
        points[:, 0] = point_cloud['x']
        points[:, 1] = point_cloud['y']
        points[:, 2] = point_cloud['z']
        points = points[np.isfinite(points).all(axis=1)]

        for point in points:
            self.tree.updateNode(tuple(point), True)

        self.tree.updateInnerOccupancy()

    def octomap_callback(self, msg):
        print("in octomap callback")
        self.occupancy_grid = self.process_octomap(msg)
        
    def decode_octomap(self, octomap_msg):
        """
        Convert a binary OctoMap message into a list of occupied 3D points.
        Note: Assumes 256x256x256 max grid size — adjust if needed.
        """
        rospy.loginfo("Decoding OctoMap...")
        resolution = octomap_msg.resolution
        occupied_points = []

        # Loop over each byte in the binary data
        for offset, byte in enumerate(octomap_msg.data):
            for bit in range(8):
                if (byte >> bit) & 1:
                    # Compute voxel index
                    voxel_index = offset * 8 + bit
                    x = (voxel_index % (256 * 256)) % 256
                    y = (voxel_index % (256 * 256)) // 256
                    z = voxel_index // (256 * 256)

                    # Convert to real-world coordinates
                    x_real = x * resolution
                    y_real = y * resolution
                    z_real = z * resolution

                    occupied_points.append((x_real, y_real, z_real))

        rospy.loginfo(f"Decoded {len(occupied_points)} occupied voxels.")
        return occupied_points   
    
    def process_octomap(self, octomap_msg):
        """
        Convert 3D OctoMap to 2D occupancy grid based on height.
        Marks cells as obstacles if height > 0.2m.
        """
        grid_size = (200, 200)
        resolution = self.grid_resolution
        rover_radius = 0  # Use actual rover radius here
        safety_margin = 0
        inflation_cells = int((rover_radius + safety_margin) / resolution)

        occupancy_grid = np.zeros(grid_size, dtype=np.float32)
        occupied_points = self.decode_octomap(octomap_msg)

        for x, y, z in occupied_points:
            grid_x = int((x - self.grid_origin[0]) / resolution)
            grid_y = int((y - self.grid_origin[1]) / resolution)

            if 0 <= grid_x < grid_size[0] and 0 <= grid_y < grid_size[1]:
               # print("Z", z)
                if z > 0.2:  # Considered an obstacle
                    for dx in range(-inflation_cells, inflation_cells + 1):
                        for dy in range(-inflation_cells, inflation_cells + 1):
                            new_x = min(max(grid_x + dx, 0), grid_size[0] - 1)
                            new_y = min(max(grid_y + dy, 0), grid_size[1] - 1)
                            occupancy_grid[new_x, new_y] = 1000  # Assign high cost
               # else: 
                #    print("not always an obstacle")
        return occupancy_grid

    def heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def get_neighbors(self, node):
        x, y = node
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < 200 and 0 <= ny < 200:
                # print("first checkpoint")
                if self.occupancy_grid[nx, ny] < 1000:
                    yield (nx, ny)

    def a_star(self, start, goal):
        print(f"A* called from {start} to {goal}")
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        visited_count = 0

        while not open_set.empty():
            _, current = open_set.get()
            visited_count += 1

            if current == goal:
                print(f"Goal reached after visiting {visited_count} nodes.")
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                #print(f"Considering neighbor: {neighbor}")
                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    open_set.put((f_score[neighbor], neighbor))
        
        print(f"Open set exhausted. Visited {visited_count} nodes. No path found.")
        return []
    
    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def publish_path(self, path):
        print("publishing path")
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        for x, y in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x * self.grid_resolution
            pose.pose.position.y = y * self.grid_resolution
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            if self.occupancy_grid is None:
                rospy.loginfo("Waiting for occupancy grid...")
                self.update_rate.sleep()
                continue

            start = (int(self.current_position[0] / self.grid_resolution),
                     int(self.current_position[1] / self.grid_resolution))
            goal = (int(self.goal[0] / self.grid_resolution), int(self.goal[1] / self.grid_resolution))
          
            path = self.a_star(start, goal)
            print("path is", path)
            if path:
                print("found path")
                self.publish_path(path)
            self.update_rate.sleep()

if __name__ == '__main__':
    try:
        planner = OctoMapAStar()
        planner.run()
    except rospy.ROSInterruptException:
        pass
