#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from octomap_msgs.msg import Octomap
from octomap import OcTree
import ros_numpy
from std_msgs.msg import Header

class ZEDToOctoMap:
    def __init__(self):
        rospy.init_node("zed_to_octomap", anonymous=True)

        # Parameters
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "/zed/point_cloud/cloud_registered")
        self.octomap_topic = rospy.get_param("~octomap_topic", "/octomap_binary")
        self.tree_resolution = rospy.get_param("~resolution", 0.1)  # OctoMap resolution (meters)

        # Publishers and Subscribers
        self.pointcloud_sub = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.pointcloud_callback)
        self.octomap_pub = rospy.Publisher(self.octomap_topic, Octomap, queue_size=10)

        # OctoMap tree
        self.tree = OcTree(self.tree_resolution)

    def pointcloud_callback(self, msg):
        """
        Callback function to process the point cloud data and update the OctoMap.
        """
        rospy.loginfo("Received point cloud, processing...")

        # Convert ROS PointCloud2 message to numpy array
        point_cloud = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
        points = np.zeros((point_cloud.shape[0], 3), dtype=np.float32)

        # Extract x, y, z values from point cloud
        points[:, 0] = point_cloud['x']
        points[:, 1] = point_cloud['y']
        points[:, 2] = point_cloud['z']

        # Filter out invalid points (NaN or Inf)
        valid_mask = np.isfinite(points).all(axis=1)
        points = points[valid_mask]

        # Update the OctoMap tree with the valid points
        for point in points:
            self.tree.updateNode(tuple(point), True)

        # Update inner occupancy probabilities
        self.tree.updateInnerOccupancy()

        # Publish the OctoMap
        self.publish_octomap()

    def publish_octomap(self):
        """
        Publish the generated OctoMap as a ROS message.
        """
        rospy.loginfo("Publishing OctoMap...")
        octomap_msg = self.tree.writeBinaryMsg()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        octomap_msg.header = header
        self.octomap_pub.publish(octomap_msg)

    def run(self):
        """
        Keep the node running and processing data.
        """
        rospy.spin()


if __name__ == "__main__":
    try:
        zed_to_octomap = ZEDToOctoMap()
        zed_to_octomap.run()
    except rospy.ROSInterruptException:
        pass