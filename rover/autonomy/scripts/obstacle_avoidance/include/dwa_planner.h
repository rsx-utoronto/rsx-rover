#ifndef DWA_PLANNER_H
#define DWA_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <tf2/utils.h>
#include "trajectory_visualizer.h"
#include "dwa_structs.h"

namespace dwa{

class DWAPlanner
{
public:
    DWAPlanner(ros::NodeHandle &nh, tf2_ros::Buffer* tfBuffer);
    
    // Parameters
    void define_parameters(ros::NodeHandle &nh);

    // Callbacks
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void tf_lookup(const std::string &target_frame, const std::string &source_frame, Pose2D &current_pose_);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // Planning
    void plan();
    double compute_cost(const std::vector<Pose2D> &traj, double v, double w);
    bool goal_reached();

    // Obstacle avoidance
    bool check_collision(const std::vector<Pose2D> &traj);
    // double calculateObstacleCost(const std::vector<Pose2D>& traj); // NOT USING THIS FOR NOW

    // Robot Model
    std::vector<std::pair<double, double>> transform_robot_footprint(const Pose2D &pose, std::vector<std::pair<double, double>> &robot_footprint);
    std::vector<std::pair<double, double>> get_robot_grid(std::vector<std::pair<double, double>> &robot_footprint);

    // Octomap
    void get_octomap_bounds(octomap::OcTree* octree);

private:

    // Structs // defined in dwa namespace in dwa_structs.h

    VelocityRange vr;
    DWAConfig dwac;
    RobotFootprint rf;
    Pose2D current_pose;
    ObstacleAvoidanceConfig oac;
    Pose2D goal;

    double current_v;
    double current_w;
    double v_max = 0.0;
    double w_max = 0.0;
    double v_min = 0.0;
    double w_min = 0.0;
    bool goal_received = false;
    double xy_goal_tolerance;
    double height_cost = 0.0;
    
    
    // ROS communication
    tf2_ros::Buffer* tfBuffer_;
    ros::Subscriber octomap_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber goal_sub;
    ros::Publisher cmd_vel_pub;
    std::shared_ptr<octomap::OcTree> octree_;
    std::string octomap_topic;
    std::string odom_topic;
    std::string goal_topic;
    std::string cmd_vel_topic;
    int rate;
    ros::NodeHandle pnh; // Private node handle

    // Visualization
    Visualizer vis;
};





} // namespace dwa

# endif // DWA_PLANNER_H
