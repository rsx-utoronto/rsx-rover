#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
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

namespace dwa{ // This namespace is defined in dwa_structs.h
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

    // Planning
    void plan();

    // Obstacle avoidance
    bool check_collision(const std::vector<Pose2D> &traj);

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

    double current_v;
    double current_w;
    
    
    // ROS communication
    tf2_ros::Buffer* tfBuffer_;
    ros::Subscriber octomap_sub;
    ros::Subscriber odom_sub;
    ros::Publisher cmd_vel_pub;
    std::shared_ptr<octomap::OcTree> octree_;
    std::string octomap_topic;
    std::string odom_topic;
    int rate;
    ros::NodeHandle pnh; // Private node handle

    // Visualization
    Visualizer vis;
};

// Constructor

DWAPlanner::DWAPlanner(ros::NodeHandle &nh, tf2_ros::Buffer* tfBuffer) 
    : pnh("~"),
      vis(pnh, "map")
{
    // Variables
    // Private node handle to get parameters from the launch file where the parameter file is loaded in the dwa_planner node
    
    DWAPlanner::define_parameters(pnh);

    
    // ROS communication
    tfBuffer_ = tfBuffer;
    // tf2_ros::TransformListener TransformListener(tfBuffer_);

    octomap_sub = nh.subscribe(octomap_topic, 1, &DWAPlanner::octomapCallback, this);
    odom_sub = nh.subscribe(odom_topic, 1, &DWAPlanner::odomCallback, this);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

// Parameters

void DWAPlanner::define_parameters(ros::NodeHandle &nh)
{
    /*
    Defines all the parameters that are input through ROS launch file and yaml file
    Also sets default values to these parameters
    They are part of the structs defined before.
    */

    // Variables
    // ros::param::get("~v_min", this->vr.v_min);

    nh.param<double>("v_min", vr.v_min, 0.0);
    nh.param<double>("v_max", vr.v_max, 2.0);
    nh.param<double>("w_min", vr.w_min, -1);
    nh.param<double>("w_max", vr.w_max, 1);
    nh.param<double>("max_accel_lin", dwac.max_accel_lin, 0.5);
    nh.param<double>("max_accel_ang", dwac.max_accel_ang, 0.2);
    nh.param<double>("dt", dwac.dt, 0.1);
    nh.param<double>("sim_time", dwac.sim_time, 2.0);
    nh.param<int>("num_samples_v", dwac.num_samples_v, 100);
    nh.param<int>("num_samples_w", dwac.num_samples_w, 100);
    nh.param<double>("w_heading", dwac.w_heading, 1.0);
    nh.param<double>("w_dist", dwac.w_dist, 1.0);
    nh.param<double>("w_vel", dwac.w_vel, 1.0);
    nh.param<double>("robot_length", rf.length, 0.5);
    nh.param<double>("robot_width", rf.width, 0.5);
    nh.param<double>("robot_height", rf.height, 0.5);
    nh.param<double>("robot_grid_n", rf.robot_grid_n, 10);
    nh.param<double>("min_z", oac.min_z, 0.0);
    nh.param<double>("max_z", oac.max_z, 1.0);

    // Initial pose set to 0 just in case the odom topic is not publishing
    current_pose.x = 0.0;
    current_pose.y = 0.0;
    current_pose.theta = 0.0;

    nh.param<int>("controller_frequency", rate, 10);

    // Topics
    nh.param<std::string>("octomap_topic", octomap_topic, "/octomap_full");
    nh.param<std::string>("odom_topic", odom_topic, "/odom");
}

void DWAPlanner::octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
{
    /* Takes the full octomap, converts to Abstract Octree message,
    casts to octree and stores it in the octree_ variable (global)
    */

    // Convert the ROS message to an octomap::AbstractOcTree
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    if (!tree)
    {
      ROS_ERROR("Failed to convert OctoMap message to OctoMap data structure");
      return;
    }

    // Try casting to an OcTree (most common)
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
    if (!octree)
    {
      ROS_ERROR("OctoMap is not an OcTree!");
      return;
    }
    ROS_INFO("Received an OcTree with %zu nodes", octree->size());

    // Store it for later use in collision checks
    octree_.reset(octree);
}

void DWAPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // Get current velocity and position
    current_v = msg->twist.twist.linear.x;
    current_w = msg->twist.twist.angular.z;

    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;
    double current_yaw = tf2::getYaw(msg->pose.pose.orientation);
    current_pose.theta = current_yaw;

    // double current_y = msg->pose.pose.position.y;
    // double current_yaw = msg->pose.pose.orientation.z;
}

void DWAPlanner::tf_lookup(const std::string &target_frame, const std::string &source_frame, Pose2D &current_pose_)
{
    /* I GIVE UP
    I don't understand tf listener in C++ yet so I am not gonna use this function (already spent a while on this)
    In future, whoever works on this code, feel free to continue this, nothing wrong with this approach */
    
    try 
    {
    geometry_msgs::TransformStamped transform 
        = tfBuffer_->lookupTransform(target_frame, source_frame, ros::Time(0));
    
    double x = transform.transform.translation.x;
    double y = transform.transform.translation.y;
    double yaw = tf2::getYaw(transform.transform.rotation);

    current_pose_.x = x;
    current_pose_.y = y;
    current_pose_.theta = yaw;
    } 
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("TF lookup failed: %s", ex.what());
    }
}


void DWAPlanner::plan()
{
    /* Combines other parts of the code and runs planning */

    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
        ROS_INFO("Planning...");

        // Need this check because the first few times the planner runs, it will not have an octomap, causing the node to crash
        if (!octree_)
        {
            ROS_WARN("No octomap received yet, skipping planning...");
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        // 1. Compute feasible velocity ranges (dynamic window)
        double v_min = std::max(this->vr.v_min, current_v - this->dwac.max_accel_lin * this->dwac.dt);
        double v_max = std::min(this->vr.v_max, current_v + this->dwac.max_accel_lin * this->dwac.dt);
        double w_min = std::max(this->vr.w_min, current_w - this->dwac.max_accel_ang * this->dwac.dt);
        double w_max = std::min(this->vr.w_max, current_w + this->dwac.max_accel_ang * this->dwac.dt);

        // 2. Discretize velocity space
        double best_cost = std::numeric_limits<double>::infinity();
        double best_v = 0.0;
        double best_w = 0.0;

        get_octomap_bounds(octree_.get());

        // 3. Iterate over all velocities
        for (int i = 0; i < this->dwac.num_samples_v; i++)
        {
            double v = v_min + i * (v_max - v_min) / this->dwac.num_samples_v;
            for (int j = 0; j < this->dwac.num_samples_w; j++)
            {
                double w = w_min + j * (w_max - w_min) / this->dwac.num_samples_w;

                // 4. Simulate trajectory

                // Pose2D current_pose;

                // Gave up on tf lookup, using odom for now
                // Target frame is map, source frame is base_link
                // Target frame is the frame in which we need coordinates, source frame is the frame in which we have coordinates
                // tf_lookup("map", "base_link", current_pose);

                Pose2D sim_pose = current_pose;
                std::vector<Pose2D> traj;

                // For every velocity, simulate the trajectory for next sim_time sec and check for collision
                // The trajectory has the same velocity for the entire sim_time
                // Find the trajectory with the least cost
                traj.push_back(sim_pose);
                for (double t = this->dwac.dt; t < this->dwac.sim_time; t += this->dwac.dt)
                {
                    // 4.1 Add to the trajectory
                    sim_pose.x += v * cos(sim_pose.theta) * this->dwac.dt;
                    sim_pose.y += v * sin(sim_pose.theta) * this->dwac.dt;
                    sim_pose.theta += w * this->dwac.dt;
                    traj.push_back(sim_pose);
                }                
                // ROS_INFO("Number of points in trajectory: %lu", traj.size());
                // 4.2 Check for collision
                if (!check_collision(traj)) // check_collision returns true if there is a collision
                {
                    vis.publishTrajectory(traj);
                }
                // 5. Compute cost
                // 6. Update best trajectory
            }
        }

        
        ros::spinOnce();
        loop_rate.sleep();
        
    }

}

bool DWAPlanner::check_collision(const std::vector<Pose2D> &traj)
{
    /* Checks if the current trajectory is collision-free 
    Returns true if there is a collision */
    for (const auto &pose : traj)
    {
        // Make a robot model around each pose in the trajectory
        // Check if any part of the robot collides with an obstacle in the octree
        // If it does, return true

        // Get the top left, top right, bottom left and bottom right points of the robot footprint
        // Assuming the centre is at (0,0) -> base_link frame
        std::vector<std::pair<double, double>> robot_footprint = {
        {rf.length/2, rf.width/2}, // Top right
        {-rf.length/2, rf.width/2}, // Top left
        {-rf.length/2, -rf.width/2}, // Bottom left
        {rf.length/2, -rf.width/2} // Bottom right
        };

        // Make the robot footprint into a grid
        std::vector<std::pair<double, double>> robot_grid = get_robot_grid(robot_footprint);

        // Transform the robot grid to the current pose
        robot_grid = transform_robot_footprint(pose, robot_grid);
        
        // if ((i % 5) == 0)
        // {
        //     ROS_INFO("Visualizing robot footprint and grid of size: %lu\n\n\n", robot_grid.size());
        //     // vis.publishRobotFootprint(robot_footprint);
        //     vis.publishRobotGrid(robot_grid);
        // }
        // i++;
        // Check if any of the grid cells are occupied in the octree

        for (const auto &cell : robot_grid)
        {
            for (double z = oac.min_z; z <= oac.max_z; z += ((oac.max_z - oac.min_z)/rf.robot_grid_n))
            {   
                
                octomap::OcTreeKey key = this->octree_->coordToKey(cell.first, cell.second, z);
                octomap::OcTreeNode* node = this->octree_->search(key);

                // A lot of nodes are not found in the octree 
                // THIS IS NORMAL (check documentation)
                // Decision: Any node that's not found is considered unoccupied
                if (node && this->octree_->isNodeOccupied(node))
                {
                    // ROS_INFO("Checking cell (%f, %f, %f)", cell.first, cell.second, z);
                    return true;
                }
            }
        }

    }

    return false;
}

std::vector<std::pair<double, double>> DWAPlanner::transform_robot_footprint(const Pose2D &pose, std::vector<std::pair<double, double>> &robot_footprint)
{

    std::vector<std::pair<double, double>> transformed_footprint;
    // Rotate the robot footprint by the current pose

    for (const auto &point : robot_footprint)
    {
        double x = point.first * cos(pose.theta) - point.second * sin(pose.theta);
        double y = point.first * sin(pose.theta) + point.second * cos(pose.theta);

        // Translate the robot footprint to the current pose
        x += pose.x;
        y += pose.y;

        transformed_footprint.push_back({x, y});

    }
    return transformed_footprint;
}

std::vector<std::pair<double, double>> DWAPlanner::get_robot_grid(std::vector<std::pair<double, double>> &robot_footprint)
{
    // Get the grid of the robot footprint
    // Divide the robot footprint into n x n grid cells
    // Return the grid cells

    std::vector<std::pair<double, double>> robot_grid;

    // Get the top left, top right, bottom left and bottom right points of the robot footprint
    std::pair<double, double> top_right = robot_footprint[0];
    std::pair<double, double> top_left = robot_footprint[1];
    std::pair<double, double> bottom_left = robot_footprint[2];
    std::pair<double, double> bottom_right = robot_footprint[3];

    // ROS_INFO("Length of top side: %f", top_right.first - top_left.first);
    // ROS_INFO("Length of right side: %f", top_right.second - bottom_right.second);

    for (double x = top_left.first; x < top_right.first; x += rf.length/rf.robot_grid_n)
    {
        for (double y = top_right.second; y > bottom_right.second; y -= rf.length/rf.robot_grid_n)
        {
            robot_grid.push_back({x, y});
        }
    }
    return robot_grid;

} 

void DWAPlanner::get_octomap_bounds(octomap::OcTree* octree)
{
    /* 
    A function to fetch the octomap bounds because InBBX and related functions don't work
    */
    double x_min, y_min, z_min;
    double x_max, y_max, z_max;
    octree->getMetricMin(x_min, y_min, z_min);
    octree->getMetricMax(x_max, y_max, z_max);

    ROS_INFO("Metric bounds: x in [%f, %f], y in [%f, %f], z in [%f, %f]",
         x_min, x_max, y_min, y_max, z_min, z_max);

}

}// namespace dwa
// Namespace is closed here before main because main is the entry point of the program
// compiler will fail to compile if the namespace is not closed before main
// main is supposed to be in global namespace


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dwa_planner");

    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    dwa::DWAPlanner planner(nh, &tfBuffer);
    planner.plan();
    ros::spin();
    return 0;
}