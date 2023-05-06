/* Subscribes to lidar to receive point cloud data */

#include "ros/ros.h"
#include "traversability_score_helpers.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Convert the sensor_msgs::PointCloud2 message to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    /* segment point cloud here */

    // calculate the traversability score
    double score = calculateTraversabilityScore(cloud);
    std::cout << "traversability score: " << score << std::endl;
}

int main(int argc, char** argv) {
    // Initialize the ROS Node with a node handler
    ros::init (argc, argv, "lidar_pcl_subscriber");
    ros::NodeHandle nh;

    // Create a ROS Subscriber to lidar with a queue_size of 1 and a callback function to cloud_cb
    ros::Subscriber sub = nh.subscribe("Lidar topic name?", 1, cloud_cb);

    ros::spin();

    // Success
    return 0;
}