#include "trajectory_visualizer.h"

namespace dwa
{
TrajectoryVisualizer::TrajectoryVisualizer(ros::NodeHandle &nh, const std::string& frame_id)
  : frame_id_(frame_id)
{
    marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("dwa_trajectories", 1);
    line_marker_pub_ = nh.advertise<visualization_msgs::Marker>("dwa_trajectory", 1);
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
}  // namespace dwa
