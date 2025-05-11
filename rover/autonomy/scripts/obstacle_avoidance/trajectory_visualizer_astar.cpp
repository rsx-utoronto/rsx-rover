#include "trajectory_visualizer_astar.h"

namespace astar
{
Visualizer::Visualizer(ros::NodeHandle &nh, const std::string& frame_id)
  : frame_id_(frame_id)
{
    line_marker_pub_ = nh.advertise<visualization_msgs::Marker>("astar_waypoints", 1);
    footprint_pub_ = nh.advertise<visualization_msgs::Marker>("robot_footprint", 1);
    grid_pub_ = nh.advertise<visualization_msgs::Marker>("robot_grid", 1);
}

void Visualizer::publishPath(const std::vector<GridNode>&path)
{
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = frame_id_;
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "astar_path";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;

    // Scale: how thick the line strip is
    line_marker.scale.x = 0.02;  // meters

    // Color RGBA
    line_marker.color.r = 1.0;
    line_marker.color.g = 0.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;   // fully opaque

    line_marker.pose.orientation.x = 0.0; 
    line_marker.pose.orientation.y = 0.0;
    line_marker.pose.orientation.z = 0.0;
    line_marker.pose.orientation.w = 1.0;  // no rotation

    // Build the list of points in the line strip
    for (const auto& node : path)
    {
      geometry_msgs::Point p;
      p.x = node.pose.x;
      p.y = node.pose.y;
      p.z = 0.0;  // or any desired height
      line_marker.points.push_back(p);
    }

    // Publish the marker
    line_marker_pub_.publish(line_marker);
}

void Visualizer::publishRobotFootprint(const std::vector<std::pair<double, double>>& footprint)
    {
        visualization_msgs::Marker footprint_marker;
        footprint_marker.header.frame_id = frame_id_;
        footprint_marker.header.stamp = ros::Time::now();
        footprint_marker.ns = "robot_footprint";
        footprint_marker.id = 0;
        footprint_marker.type = visualization_msgs::Marker::LINE_STRIP;
        footprint_marker.action = visualization_msgs::Marker::ADD;

        footprint_marker.scale.x = 0.05; // Line width

        // Set color (blue)
        footprint_marker.color.r = 0.0;
        footprint_marker.color.g = 0.0;
        footprint_marker.color.b = 1.0;
        footprint_marker.color.a = 1.0;

        // To prevent quaternion errors
        footprint_marker.pose.orientation.x = 0.0; 
        footprint_marker.pose.orientation.y = 0.0;
        footprint_marker.pose.orientation.z = 0.0;
        footprint_marker.pose.orientation.w = 1.0;  // no rotation

        // Add points to form the polygon (close the loop)
        for (const auto& point : footprint)
        {
            geometry_msgs::Point p;
            p.x = point.first;
            p.y = point.second;
            p.z = 0.0;
            footprint_marker.points.push_back(p);
        }
        // Close the loop by adding the first point at the end
        if (!footprint.empty())
        {
            geometry_msgs::Point p;
            p.x = footprint[0].first;
            p.y = footprint[0].second;
            p.z = 0.0;
            footprint_marker.points.push_back(p);
        }

        footprint_pub_.publish(footprint_marker);
    }

void Visualizer::publishRobotGrid(const std::vector<std::pair<double, double>>& grid_points)
    {
        visualization_msgs::Marker grid_marker;
        grid_marker.header.frame_id = frame_id_;
        grid_marker.header.stamp = ros::Time::now();
        grid_marker.ns = "robot_grid";
        grid_marker.id = 0;
        grid_marker.type = visualization_msgs::Marker::POINTS;
        grid_marker.action = visualization_msgs::Marker::ADD;

        // Set the width and height of each point
        grid_marker.scale.x = 0.05; // Width of the points
        grid_marker.scale.y = 0.05; // Height of the points

        // Set color (green)
        grid_marker.color.r = 0.0;
        grid_marker.color.g = 1.0;
        grid_marker.color.b = 0.0;
        grid_marker.color.a = 1.0;

        // Add points
        for (const auto& point : grid_points)
        {
            geometry_msgs::Point p;
            p.x = point.first;
            p.y = point.second;
            p.z = 0.0; // Adjust if needed
            grid_marker.points.push_back(p);
        }

        grid_pub_.publish(grid_marker);
    }

}  // namespace dwa