#pragma once

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <stdlib.h>

// Calculates the traversabiliy score from a point cloud
double calculateTraversabilityScore(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);


// Calculate slope from a plane
double calculateSlopeFromPlane(double a, double b, double c, double d);


// Calculate standard deviation of a set of points from a plane
double calculateStdDevFromPlane(double a, double b, double c, double d, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);


// Call ransac algorithm on the point cloud
std::pair<pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr> ransac_on_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);




