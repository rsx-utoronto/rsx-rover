// astar_structs.h
// These are called Guards, they play a crucial role in preventing multiple inclusions of the same header file, 
// which can lead to various compilation errors, such as the redefinition of classes, structs, functions, or variables.
// ifndef - if not defined 
// define - define the symbol
// if already defined, skip all the lines and go to endif
// #pragma once // This is another way to prevent multiple inclusions
#ifndef ROVER_ASTAR_STRUCTS_H
#define ROVER_ASTAR_STRUCTS_H
#include <cmath>    // for std::abs



namespace astar // These structs will only be used in the astar namespace
// This is a way to organize the code and make it more reausable
// And to prevent name clashes
{
    /**
     * @struct Pose2D
     * @brief Represents a 2D pose with x, y coordinates and orientation theta.
     */
    struct Pose2D
    {
        double x;      ///< X-coordinate
        double y;      ///< Y-coordinate
        double theta;  ///< Yaw in radians
    };

    /**
     * @struct AstarConfig
     * @brief Configuration parameters for the Dynamic Window Approach planner.
     */
    struct AstarConfig
    {
        double grid_resolution;  ///< Resolution of the grid
    };

    /**
     * @struct RobotFootprint
     * @brief Physical parameters of the robot (Assuming robot is a cuboid).
     */
    struct RobotFootprint
    {
        double width;      ///< Width of the robot
        double length;     ///< Length of the robot
        double height;     ///< Height of the robot
        double robot_grid_n;  ///< Resolution for collision checking
        /// The robot is approximated as a cuboid with the given dimensions
        /// The cuboid is divided into a grid of cells with the given resolution
    };

    /**
     * @struct Obstacle Avoidance Config
     * @brief Parameters defining the obstacle avoidance behavior
     */

    struct ObstacleAvoidanceConfig
    {
        double min_z;  ///< Minimum height of obstacles to consider
        double max_z;  ///< Maximum height of obstacles to consider
    };
    /**
     * @struct GridNode
     * @brief Represents a node in the grid for A* pathfinding.
     */
    struct GridNode 
    {
        Pose2D pose;
        double g;
        double f;
        GridNode* parent;

        bool operator<(GridNode const& o) const 
        { 
            if (std::fabs(f - o.f) < 1e-6) return g < o.g;  // prefer deeper g
            return f > o.f; 
        }

        bool operator==(GridNode const& o) const 
        {
            static constexpr double eps = 1e-6;
            return std::abs(pose.x - o.pose.x) < eps
                && std::abs(pose.y - o.pose.y) < eps;
        }
    };

}

#endif // ROVER_ASTAR_STRUCTS_H
