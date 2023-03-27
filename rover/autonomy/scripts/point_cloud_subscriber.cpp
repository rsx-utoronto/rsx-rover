/* Subscribes to lidar to receive point cloud data */

#include "ros/ros.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.1, 0.1, 0.1);
    sor.filter (cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(cloud_filtered, output);
}

int main(int argc, char** argv) {
    // Initialize the ROS Node
    ros::init (argc, argv, "lidar_point_cloud");
    ros::NodeHandle nh;

    // Print "Hello" message with node name to the terminal and ROS log file
    ROS_INFO_STREAM("Preparing to receive point cloud " << ros::this_node::getName());

    // Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function to cloud_cb
    ros::Subscriber sub = nh.subscribe("Lidar topic name????", 1, cloud_cb);

    // Spin
    ros::spin();

    // Success
    return 0;
}