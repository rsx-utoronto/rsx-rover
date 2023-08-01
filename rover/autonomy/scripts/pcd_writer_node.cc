#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <thread>

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

int main(int argc, char** argv) {

    // Initialize the ROS Node with a node handler
    ros::init (argc, argv, "pcd_writer");
    ros::NodeHandle nh;

    // ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloud_cb);
    // ros::Subscriber sub = nh.subscribe("/ouster/points", 1, cloud_cb);
    ros::Subscriber sub = nh.subscribe("/recent_cloud", 1, cloud_cb);
    
    ros::spin();

    // Success
    return 0;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    
    // Convert the sensor_msgs::PointCloud2 message to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
    // Print Number of poins in point cloud
    // std::cout <<  cloud->width << "\n"; 

    // Measure time elapsed for conversion 
    auto start = std::chrono::high_resolution_clock::now();

    pcl::io::savePCDFileASCII ("/home/arwin/rover_ws/src/rsx-rover/rover/autonomy/tests/pcd/test_pcd.pcd", *cloud);

    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    std::cout << duration << " ms\n";


}