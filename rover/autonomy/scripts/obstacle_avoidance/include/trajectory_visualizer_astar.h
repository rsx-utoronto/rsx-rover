#ifndef TRAJECTORY_VISUALIZER_ASTAR_H
#define TRAJECTORY_VISUALIZER_ASTAR_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include "astar_structs.h"

namespace astar
{
class Visualizer
{
public:
    Visualizer(ros::NodeHandle &nh, const std::string& frame_id = "map");

    /**
     * @brief Publish multiple nodes as LINE_STRIP markers in RViz.
     * @param path A vector of nodes
     */
    void publishPath(const std::vector<std::vector<GridNode>>& path);

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
    ros::Publisher line_marker_pub_;
    ros::Publisher footprint_pub_;
    ros::Publisher grid_pub_;
    std::string frame_id_;
};
}
#endif // TRAJECTORY_VISUALIZER_H
