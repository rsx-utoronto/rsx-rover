/* Subscribes to lidar to receive point cloud data */

#include "ros/ros.h"
#include "segment_pcl.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

#include "pcl_traversibility_score.h"

ros::Publisher obstacle_cells;
const int dim = 40;
segmented_pcl p;
ros::Publisher pub;

nav_msgs::OccupancyGrid pub_grid()
{
    nav_msgs::OccupancyGrid ransac_grid;

    ransac_grid.info.width = dim;
    ransac_grid.info.height = dim;
    ransac_grid.info.resolution = (p.x_grid_sz + p.y_grid_sz) / 2; //resolution needs to be fixed (width dne height)

    ransac_grid.info.origin.position.x = p.origin_x;
    ransac_grid.info.origin.position.y = p.origin_y;

    ransac_grid.data.resize(dim*dim);

    int n = 0;
    for (int i = 0; i < dim; i++)
    {
        for (int j = 0; j < dim; j++)
        {
            std::cout <<p.grid[i][j] << std::endl;
            ransac_grid.data[n] = p.grid[i][j];
            n++;
        }
    }

    return ransac_grid;
}


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    
    // Convert the sensor_msgs::PointCloud2 message to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // segment point cloud & calculate the traversability score

    p = segment_pcl(cloud, dim);
    
    pub.publish(pub_grid());

}

int main(int argc, char** argv) {

    // Initialize the ROS Node with a node handler
    ros::init (argc, argv, "lidar_pcl_subscriber");
    ros::NodeHandle nh;

    // ros::Subscriber sub = nh.subscribe("/ouster/points", 1, cloud_cb);
    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloud_cb);
    
    pub = nh.advertise<nav_msgs::OccupancyGrid>("/ransac_grid", 1);

    ros::spin();

    // Success
    return 0;
}