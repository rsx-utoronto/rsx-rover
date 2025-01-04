#include "trajectory_visualizer.h"
#include "dwa_structs.h"
#include "dwa_planner.h"

namespace dwa{ // This namespace is defined in dwa_structs.h

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
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
    goal_sub = nh.subscribe(goal_topic, 1, &DWAPlanner::goalCallback, this);
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
    nh.param<double>("w_lin", dwac.w_lin, 1.0);
    nh.param<double>("w_ang", dwac.w_ang, 1.0);
    nh.param<double>("w_obs", dwac.w_obs, 1.0);
    nh.param<double>("robot_length", rf.length, 0.5);
    nh.param<double>("robot_width", rf.width, 0.5);
    nh.param<double>("robot_height", rf.height, 0.5);
    nh.param<double>("robot_grid_n", rf.robot_grid_n, 10);
    nh.param<double>("min_z", oac.min_z, 0.0);
    nh.param<double>("max_z", oac.max_z, 1.0);
    nh.param<double>("obstacle_threshold", oac.obstacle_threshold, 0.5);
    nh.param<double>("xy_goal_tolerance", xy_goal_tolerance, 0.1);

    // Initial pose set to 0 just in case the odom topic is not publishing
    current_pose.x = 0.0;
    current_pose.y = 0.0;
    current_pose.theta = 0.0;

    goal.x = 0.0;
    goal.y = 0.0;
    goal.theta = 0.0;

    nh.param<int>("controller_frequency", rate, 10);

    // Topics
    nh.param<std::string>("octomap_topic", octomap_topic, "/octomap_full");
    nh.param<std::string>("odom_topic", odom_topic, "/odom");
    nh.param<std::string>("goal_topic", goal_topic, "/goal");
    nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "/cmd_vel");
}

// Callbacks
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

void DWAPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Get the goal position
    this->goal_received = true;
    ROS_INFO("Goal received");
    goal.x = msg->pose.position.x;
    goal.y = msg->pose.position.y;
    goal.theta = tf2::getYaw(msg->pose.orientation);
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
        if (!goal_reached())
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
            if (!(this->goal_received))
            {
                ROS_WARN("No goal received yet, skipping planning...");
                ros::spinOnce();
                loop_rate.sleep();
                continue;
            }

            // 1. Compute feasible velocity ranges (dynamic window)
            this->v_min = std::max(this->vr.v_min, current_v - this->dwac.max_accel_lin * 1/this->rate);
            this->v_max = std::min(this->vr.v_max, current_v + this->dwac.max_accel_lin * 1/this->rate);
            this->w_min = std::max(this->vr.w_min, current_w - this->dwac.max_accel_ang * 1/this->rate);
            this->w_max = std::min(this->vr.w_max, current_w + this->dwac.max_accel_ang * 1/this->rate);

            // 2. Discretize velocity space
            double best_cost = std::numeric_limits<double>::infinity();
            double best_v = 0.0;
            double best_w = 0.0;
            std::vector<Pose2D> best_traj;
            best_traj.push_back(current_pose);

            // get_octomap_bounds(octree_.get());

            // 3. Iterate over all velocities
            for (int i = 0; i < this->dwac.num_samples_v; i++)
            {
                double v = this->v_min + i * (this->v_max - this->v_min) / this->dwac.num_samples_v;
                for (int j = 0; j < this->dwac.num_samples_w; j++)
                {
                    double w = this->w_min + j * (this->w_max - this->w_min) / this->dwac.num_samples_w;

                    // 4. Simulate trajectory

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

                        // 5. Compute cost

                        // Less cost is better
                        double cost = 0.0;
                        cost = compute_cost(traj, v, w);

                        // 6. Update best trajectory
                        if (cost < best_cost)
                        {
                            best_cost = cost;
                            best_v = v;
                            best_w = w;
                            best_traj = traj;
                        }

                    }                
                }
            }
            if (best_cost == std::numeric_limits<double>::infinity())
            {
                ROS_WARN("No feasible trajectory found");
            }
            else
            {
                ROS_INFO("Best cost: %f", best_cost);
                ROS_INFO("Best linear velocity: %f", best_v);
                ROS_INFO("Best angular velocity: %f", best_w);
                vis.publishTrajectory(best_traj);
            }
            
            // 7. Publish the best velocity
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = best_v;
            cmd_vel.angular.z = best_w;
            cmd_vel_pub.publish(cmd_vel);
        }
        else
        {
            ROS_INFO("Goal reached");
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub.publish(cmd_vel);
            ROS_INFO("Waiting for a different goal...\n\n\n\n");
            while (ros::ok() && goal_reached())
            {
                ros::spinOnce();
                loop_rate.sleep();
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

    // // Just for debugging
    // int a = 0;
    // int b = 0;
    // int c = 0;

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
                // a++;
                // ROS_INFO("Checking cell %d", a);
                octomap::OcTreeKey key = this->octree_->coordToKey(cell.first, cell.second, z);
                octomap::OcTreeNode* node = this->octree_->search(key);

                // A lot of nodes are not found in the octree 
                // THIS IS NORMAL (check documentation)
                // Decision: Any node that's not found, check at a coarser depth (until 5 levels up)

                // // Just for debugging, checking how many nodes were found before and after depth check
                // if (node)
                // {
                //     c++;
                //     ROS_INFO("Found %d nodes before depth check", c);
                // }
                if (!node)
                {
                    unsigned int maxDepth = octree_->getTreeDepth();
                    for (unsigned int d = maxDepth - 1; d > maxDepth - 5; d = d - 1)
                    {
                        node = octree_->search(key, d);
                        if (node) 
                        {
                            // ROS_INFO("Found a coarser node at depth = %u", d);
                            break;
                        }
                    }
                }
                // if (node)
                // {
                //     b++;
                //     ROS_INFO("Found %d nodes", b);
                // }
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

double DWAPlanner::compute_cost(const std::vector<Pose2D> &traj, double v, double w)
{
    /* 

    Computes the cost of the trajectory based on the cost function 
    Metrics:
    1. Distance of the final point* in trajectory from goal
    2. Heading difference of the final point* as compared to goal
    3. Velocity cost:
        a) Higher linear velocity is better
        b) Average angular velocity is better
    4. Distance from nearest obstacle: Not using this for now

    *final point is the last point in the trajectory

    */
    // 1. Distance Cost
    Pose2D final_pose = traj.back();
    double dx = goal.x - final_pose.x;
    double dy = goal.y - final_pose.y;
    double dist_cost = sqrt(pow(dx, 2) + pow(dy, 2));
    
    // 2. Heading Cost
    double heading_cost = std::fabs(std::atan2(dy, dx) - final_pose.theta);

    // 3. Velocity Cost
    // 3.1 Linear velocity cost
    double v_cost = 0.0;
    if (v>=0)
    {
        v_cost = std::fabs(this->v_max - v);
    }
    else
    {
        v_cost = std::fabs(this->v_min - v);
    }
    // 3.2 Angular velocity cost
    double w_cost = 0.0;
    if (w_min >= 0)
    {
        double w_avg = (w_min + w_max) / 2;
        w_cost = std::fabs(w_avg - w);
    }
    else
    {
        if (w >= 0)
        {
            double w_avg = (w_max) / 2;
            w_cost = std::fabs(w_avg - w);
        }
        else
        {
            double w_avg = (w_min) / 2;
            w_cost = std::fabs(w_min - w);
        }
    }

    // 4. Obstacle distance cost // NOT USING THIS FOR NOW
    // double obstacle_cost = calculateObstacleCost(traj);

    // Total cost
    double tot_cost = this->dwac.w_heading * heading_cost + this->dwac.w_dist * dist_cost + this->dwac.w_lin * v_cost + this->dwac.w_ang * w_cost; // + this->dwac.w_obs * obstacle_cost;

    return tot_cost;
}

bool DWAPlanner::goal_reached()
{
    // Check if the goal is reached
    double dx = goal.x - current_pose.x;
    double dy = goal.y - current_pose.y;
    double dist = sqrt(pow(dx, 2) + pow(dy, 2));
    if (dist < xy_goal_tolerance)
    {
        return true;
    }
    return false;
}

/* NOT USING THIS FOR NOW

double DWAPlanner::calculateObstacleCost(const std::vector<Pose2D>& traj) {
    NOT USING THIS FOR NOW
    
    This function is supposed to calculate the cost of a trajectory 
    based on how far it is from the nearest obstacle.
    Apparently getMetricMinDistance does not exist and there's no function which does this directly
    So I am not using this cost for now
    

    double min_distance = std::numeric_limits<double>::infinity();
    double obstacle_cost = 0.0;

    for (const auto& pose : traj) {
        // Query the nearest obstacle distance from the current pose
        octomap::point3d query_point(pose.x, pose.y, 0.0); // Assume Z=0 for simplicity
        double nearest_distance = octree_->getMetricMinDistance(query_point);

        if (nearest_distance >= 0) {
            min_distance = std::min(min_distance, nearest_distance);
        }
    }

    // Compute obstacle cost based on minimum distance
    // Obstacle cost would be 0 if the trajectory is far enough (threshold) from obstacles
    if (min_distance < oac.obstacle_threshold) {
        obstacle_cost = 1.0 / (min_distance + 1e-3); // Add epsilon to avoid division by zero
    }

    return obstacle_cost;
} 
*/



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
    return 0;
}