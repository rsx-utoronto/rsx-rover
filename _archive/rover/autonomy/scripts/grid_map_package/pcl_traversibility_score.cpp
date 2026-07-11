#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <stdlib.h>
#include <cmath>


// Calculate slope from a plane
double calculateSlopeFromPlane(double a, double b, double c, double d) {
    double magnitudeOfNormal = std::sqrt(a*a + b*b + c*c);


    // Normalize coefficients
    a = a / magnitudeOfNormal;
    b = b / magnitudeOfNormal;
    c = c / magnitudeOfNormal;
    d = d / magnitudeOfNormal;


    double slopeAngle = std::acos(c / magnitudeOfNormal);


    return slopeAngle;
}


// Calculate the accumulated standard deviation of the plane by measuring the standard deviation of each point from the plane
double calculateStdDevFromPlane(double a, double b, double c, double d, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    double stdDev = 0.0;
    int numPoints = cloud->points.size();
    double sumDistanceSquared = 0.0;


    auto points = cloud->points;


    // Accumulate the squared distances of all points from the plane
    for (int idx=0; idx<numPoints; idx++) {
        auto point = points[idx];
        double distance = std::abs(a * point.x + b * point.y + c * point.z + d) / std::sqrt(a*a + b*b + c*c);
        sumDistanceSquared += distance * distance;
    }


    // Calculate standard deviation
    double meanSquaredDistance = sumDistanceSquared / numPoints;
    stdDev = std::sqrt(meanSquaredDistance);


    return stdDev;
}


// Ransac function on a pcl PointCloud
std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> ransac_on_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) 
{


  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  
  //Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setDistanceThreshold (0.01);
  seg.setDistanceThreshold (0.1); // <-- play around with threshold


  seg.setInputCloud (cloud);


  // Computes the model
  seg.segment (*inliers, *coefficients);


  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
    return {coefficients, inliers};
  }


  // std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
  //                                     << coefficients->values[1] << " "
  //                                     << coefficients->values[2] << " " 
  //                                     << coefficients->values[3] << std::endl;


  // std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  // for (const auto& idx: inliers->indices)
  //   std::cerr << idx << "    " << cloud->points[idx].x << " "
  //                              << cloud->points[idx].y << " "
  //                              << cloud->points[idx].z << std::endl;


  return {coefficients, inliers};
}


// Calculates the traversabiliy score from a point cloud
double calculateTraversabilityScore(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    // call ransac on the generated point cloud
    int numPoints = cloud->points.size();
    std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> result = ransac_on_cloud(cloud);
    // ****need a condition to check if the ransac was successful -> generated valid plane***


    // Get coefficients of the plane
    double a = result.first->values[0];
    double b = result.first->values[1];
    double c = result.first->values[2];
    double d = result.first->values[3];


    // Calculate parameters from the ransac plane
    double standardDeviation = calculateStdDevFromPlane(a, b, c, d, cloud);


    double traversabilityScore = standardDeviation;
    return traversabilityScore;


}