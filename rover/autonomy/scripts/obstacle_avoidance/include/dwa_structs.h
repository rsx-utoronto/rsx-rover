// dwa_structs.h
// These are called Guards, they play a crucial role in preventing multiple inclusions of the same header file, 
// which can lead to various compilation errors, such as the redefinition of classes, structs, functions, or variables.
// ifndef - if not defined 
// define - define the symbol
// if already defined, skip all the lines and go to endif
// #pragma once // This is another way to prevent multiple inclusions
#ifndef ROVER_DWA_STRUCTS_H
#define ROVER_DWA_STRUCTS_H



namespace dwa // These structs will only be used in the dwa namespace
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
     * @struct VelocityRange
     * @brief Defines the minimum and maximum linear and angular velocities.
     */
    struct VelocityRange
    {
        double v_min;  ///< Minimum linear velocity
        double v_max;  ///< Maximum linear velocity
        double w_min;  ///< Minimum angular velocity
        double w_max;  ///< Maximum angular velocity
    };

    /**
     * @struct DWAConfig
     * @brief Configuration parameters for the Dynamic Window Approach planner.
     */
    struct DWAConfig
    {
        double max_accel_lin;  ///< Maximum linear acceleration
        double max_accel_ang;  ///< Maximum angular acceleration
        double dt;             ///< Time step for simulation
        double sim_time;       ///< Simulation horizon
        int num_samples_v;     ///< Number of linear velocity samples
        int num_samples_w;     ///< Number of angular velocity samples

        // Cost function weights
        double w_heading;      ///< Weight for heading cost
        double w_dist;         ///< Weight for distance cost
        double w_vel;          ///< Weight for velocity cost
    };
}

#endif // ROVER_DWA_STRUCTS_H
