#ifndef TRAJECTORY_VISUALIZER_H
#define TRAJECTORY_VISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include "dwa_structs.h"

namespace dwa
{
class TrajectoryVisualizer
{
public:
    TrajectoryVisualizer(ros::NodeHandle &nh, const std::string& frame_id = "map");

    /**
     * @brief Publish multiple trajectories as LINE_STRIP markers in RViz.
     * @param trajectories A vector of trajectories, where each trajectory
     *        is itself a vector of Pose2D positions.
     */
    void publishTrajectories(const std::vector<std::vector<Pose2D>>& trajectories);
    void publishTrajectory(const std::vector<Pose2D>& trajectory);

private:
    ros::Publisher marker_array_pub_;
    ros::Publisher line_marker_pub_;
    std::string frame_id_;
};
}
#endif // TRAJECTORY_VISUALIZER_H
