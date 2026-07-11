#include "segment_pcl.h"
segmented_pcl segment_pcl(const pcl::PointCloud<pcl::PointXYZ>::Ptr rover_cloud, int local_map_dim)
{
    using namespace pcl;


    //get min_max x and y of input cloud to segment
    pcl::PointXYZ min;
    pcl::PointXYZ max;
    pcl::getMinMax3D (*rover_cloud, min, max);
    double x_sz = max.x - min.x;
    double y_sz = max.y - min.y;
    double x_grid_sz = x_sz / local_map_dim;
    double y_grid_sz = y_sz / local_map_dim;

    double origin_x = min.x + (x_sz / 2);
    double origin_y = min.y + (y_sz / 2);


    // // Translate crop box up by 1
    // cropBoxFilter.setTranslation(Eigen::Vector3f(0, 1, 0));

    std::vector<std::vector<double>> occupancy_grid (local_map_dim);
    for (int i = 0; i < local_map_dim; i++)
    {
        occupancy_grid[i].resize(local_map_dim);
    }

    for (int i = 0; i < local_map_dim; i++)
    {
        for (int j = 0; j < local_map_dim; j++)
        {

            CropBox<PointXYZ> cropBoxFilter (true);
            cropBoxFilter.setInputCloud (rover_cloud);

            //order is bottom left --> top right

            Eigen::Vector4f min_pt (min.x + (x_grid_sz * j), min.y + (y_grid_sz * i), -1000.0f, -1000.0f);
            Eigen::Vector4f max_pt (min.x + (x_grid_sz * (j + 1)), min.y + (y_grid_sz * (i + 1)), 1000.0f, 1000.0f);

            cropBoxFilter.setMin (min_pt);
            cropBoxFilter.setMax (max_pt);

            // Cloud
            PointCloud<PointXYZ> cloud_out;
            cropBoxFilter.filter (cloud_out);

            double score = 100; //lower is better, if too little points in this cell to compute make the score really high by default
            if (cloud_out.points.size() > 10)
            {
                PointCloud<PointXYZ>::Ptr cloud_out_ptr(new pcl::PointCloud<pcl::PointXYZ>);
                *cloud_out_ptr = cloud_out;
                score = calculateTraversabilityScore(cloud_out_ptr);

            }
            occupancy_grid[j][local_map_dim - i -1] = score;

        }
    }

    segmented_pcl p;
    p.grid = occupancy_grid;
    p.x_grid_sz = x_grid_sz;
    p.y_grid_sz = y_grid_sz;
    p.x_sz = x_sz;
    p.y_sz = y_sz;
    p.origin_x = origin_x;
    p.origin_y = origin_y;
    
    return p;
}
//translates all points to local map frame
void translate_rover_to_local(pcl::PointCloud<pcl::PointXYZ> &rover_cloud, pcl::PointXYZ local_origin)
{
    for (auto &point : rover_cloud)
    {
        point.x = point.x - local_origin.x;
        point.y = point.y - local_origin.y;
    }
}