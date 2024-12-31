#ifndef TRAJECTORY_VISUALIZER_H
#define TRAJECTORY_VISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include "dwa_structs.h"

namespace dwa
{
class Visualizer
{
public:
    Visualizer(ros::NodeHandle &nh, const std::string& frame_id = "map");

    /**
     * @brief Publish multiple trajectories as LINE_STRIP markers in RViz.
     * @param trajectories A vector of trajectories, where each trajectory
     *        is itself a vector of Pose2D positions.
     */
    void publishTrajectories(const std::vector<std::vector<Pose2D>>& trajectories);

    /**
     * @brief Publish a single trajectory as a LINE_STRIP marker in RViz.
     * @param trajectory A vector of Pose2D positions.
     */
    void publishTrajectory(const std::vector<Pose2D>& trajectory);

    /**
     * @brief Publish a robot footprint as a LINE_STRIP marker in RViz.
     * @param footprint A vector of points (pairs) representing the robot footprint.
     */
    void publishRobotFootprint(const std::vector<std::pair<double, double>>& footprint);

    /**
     * @brief Publish a the robot footprint as a grid of points in RViz, used for collision checking.
     * @param grid_points A vector of points (pairs) representing the grid.
     */

    void publishRobotGrid(const std::vector<std::pair<double, double>>& grid_points);


private:
    ros::Publisher marker_array_pub_;
    ros::Publisher line_marker_pub_;
    ros::Publisher footprint_pub_;
    ros::Publisher grid_pub_;
    std::string frame_id_;
};
}
#endif // TRAJECTORY_VISUALIZER_H
