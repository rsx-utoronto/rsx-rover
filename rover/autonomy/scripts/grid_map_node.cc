
#include <ros/ros.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"
#include <fstream>
#include <iostream>

void gridmap_callback (const grid_map_msgs::GridMap map_msg);


int main(int argc, char** argv) {

    // Initialize the ROS Node with a node handler
    ros::init (argc, argv, "grid_map_subscriber");
    ros::NodeHandle nh;

    // ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloud_cb);
    // ros::Subscriber sub = nh.subscribe("/ouster/points", 1, cloud_cb);
    ros::Subscriber sub = nh.subscribe("/grid_map_filter_demo/filtered_map", 1, gridmap_callback);
    
    // Create costmap2d.
    // costmap_2d::Costmap2D costmap;
    // this->costmap2dConverter_.initializeFromGridMap(this->gridMap_, costmap);
    
    ros::spin();

    // Success
    return 0;
}

void gridmap_callback (const grid_map_msgs::GridMap map_msg) {

    std::cout << map_msg << "\n";

}