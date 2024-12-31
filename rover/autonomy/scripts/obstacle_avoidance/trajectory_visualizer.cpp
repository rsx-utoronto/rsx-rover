#include "trajectory_visualizer.h"

namespace dwa
{
TrajectoryVisualizer::TrajectoryVisualizer(ros::NodeHandle &nh, const std::string& frame_id)
  : frame_id_(frame_id)
{
    marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("dwa_trajectories", 1);
    line_marker_pub_ = nh.advertise<visualization_msgs::Marker>("dwa_trajectory", 1);
    footprint_pub_ = nh.advertise<visualization_msgs::Marker>("robot_footprint", 1);
}

void TrajectoryVisualizer::publishTrajectories(const std::vector<std::vector<Pose2D>>& trajectories)
{
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.reserve(trajectories.size());  // optional optimization

    // We'll assign one marker per trajectory (LINE_STRIP)
    int marker_id = 0;
    for (size_t i = 0; i < trajectories.size(); ++i)
    {
        const auto& traj = trajectories[i];

        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = frame_id_;
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "dwa_trajectories";
        line_marker.id = marker_id++;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;

        // Scale: how thick the line strip is
        line_marker.scale.x = 0.02;  // meters

        // Color RGBA
        line_marker.color.r = 1.0;
        line_marker.color.g = 0.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 1.0;   // fully opaque

        // Build the list of points in the line strip
        for (const auto& pose : traj)
        {
            geometry_msgs::Point p;
            p.x = pose.x;
            p.y = pose.y;
            p.z = 0.0;  // or any desired height
            line_marker.points.push_back(p);
        }

        marker_array.markers.push_back(line_marker);
    }

    // Publish the whole array of markers
    marker_array_pub_.publish(marker_array);
}

void TrajectoryVisualizer::publishTrajectory(const std::vector<Pose2D>& trajectory)
{
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = frame_id_;
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "dwa_trajectories";
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

    // Build the list of points in the line strip
    for (const auto& pose : trajectory)
    {
      geometry_msgs::Point p;
      p.x = pose.x;
      p.y = pose.y;
      p.z = 0.0;  // or any desired height
      line_marker.points.push_back(p);
    }

    // Publish the marker
    line_marker_pub_.publish(line_marker);
}

void TrajectoryVisualizer::publishRobotFootprint(const std::vector<std::pair<double, double>>& footprint)
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

}  // namespace dwa
