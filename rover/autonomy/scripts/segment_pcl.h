#pragma once

#include <iostream>
#include <vector>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/impl/point_types.hpp>

#include "pcl_traversibility_score.h"

void segment_pcl(const pcl::PointCloud<pcl::PointXYZ>::Ptr rover_cloud, int local_map_dim);
void translate_rover_to_local(pcl::PointCloud<pcl::PointXYZ> &rover_cloud, pcl::PointXYZ local_origin);


//Example main deonstrating segment_pcl function
/*
int main(void)
{
    using namespace pcl;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pcd(new pcl::PointCloud<pcl::PointXYZ>);

    // if (pcl::io::loadPCDFile<pcl::PointXYZ> ("p213.pcd", *pcd) == -1) //* load the file
    // {
    //     PCL_ERROR("Couldn't read file p213.pcd \n");
    //     return (-1);
    // }
    // std::cout << "Loaded "
    //           << pcd->width * pcd->height
    //           << " data points " << std::endl;

    // Create the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data with randomly generated set of 15 points
    cloud->width = 15;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    // Generate the data
    srand(time(0));
    for (auto &point : *cloud)
    {
        point.x = 1024 * rand() % 9;
        point.y = 1024 * rand() % 9;
        point.z = 1024 * rand() % 9;
    }

    std::cout << "Point cloud data: " << cloud->size() << " points" << std::endl;
    for (const auto &point : *cloud)
    {
        std::cout << "    " << point.x << " "
                  << point.y << " "
                  << point.z << std::endl;
    }

    segment_pcl(cloud);
    return 0;
}
*/