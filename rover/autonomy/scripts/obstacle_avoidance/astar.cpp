#include "trajectory_visualizer_astar.h"
#include "astar_structs.h"
#include "astar.h"

namespace astar{ // This namespace is defined in astar_structs.h

// Constructor

Astar::Astar(ros::NodeHandle &nh, tf2_ros::Buffer* tfBuffer) 
    : pnh("~"),
      vis(pnh, "map")
{
    // Variables
    // Private node handle to get parameters from the launch file where the parameter file is loaded in the dwa_planner node
    
    Astar::define_parameters(pnh);
    
    // ROS communication
    tfBuffer_ = tfBuffer;
    // tf2_ros::TransformListener TransformListener(tfBuffer_);

    octomap_sub = nh.subscribe(octomap_topic, 1, &Astar::octomapCallback, this);
    odom_sub = nh.subscribe(odom_topic, 1, &Astar::odomCallback, this);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
    goal_sub = nh.subscribe(goal_topic, 1, &Astar::goalCallback, this);
}

// Parameters

void Astar::define_parameters(ros::NodeHandle &nh)
{
    /*
    Defines all the parameters that are input through ROS launch file and yaml file
    Also sets default values to these parameters
    They are part of the structs defined before.
    */

    // Variables
    // ros::param::get("~v_min", this->vr.v_min);

    
    nh.param<double>("grid_resolution", ac.grid_resolution, 1.0);
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

    nh.param<int>("planner_frequency", rate, 10);

    // Topics
    nh.param<std::string>("octomap_topic", octomap_topic, "/octomap_full");
    nh.param<std::string>("odom_topic", odom_topic, "/odom");
    nh.param<std::string>("goal_topic", goal_topic, "/goal");
    nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "/cmd_vel");
}

// Callbacks
void Astar::octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
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

void Astar::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
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

void Astar::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Get the goal position
    this->goal_received = true;
    ROS_INFO("Goal received");
    goal.x = msg->pose.position.x;
    goal.y = msg->pose.position.y;
    goal.theta = tf2::getYaw(msg->pose.orientation);
}

void Astar::tf_lookup(const std::string &target_frame, const std::string &source_frame, Pose2D &current_pose_)
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

double Astar::find_node_height(const GridNode &node)
{
    /* Returns the height of the current node */
    // Make a robot grid for parent
    // Check if any part of the robot collides with an obstacle in the octree

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
    robot_grid = transform_robot_footprint(node.pose, robot_grid);

    // Check if any of the grid cells are occupied in the octree
    // Max height of the obstacle in the cells of the robot footprint
    double max_z = -std::numeric_limits<double>::infinity();
    for (const auto &cell : robot_grid)
    {          
        for (double z = oac.min_z; z <= oac.max_z; z += ((oac.max_z - oac.min_z)/rf.robot_grid_n))
        {   
            octomap::OcTreeKey key = this->octree_->coordToKey(cell.first, cell.second, z);
            octomap::OcTreeNode* node = this->octree_->search(key);

            // A lot of nodes are not found in the octree 
            // THIS IS NORMAL (check documentation)
            // Decision: Any node that's not found, check at a coarser depth (until 5 levels up)

            if (!node)
            {
                unsigned int maxDepth = octree_->getTreeDepth();
                for (unsigned int d = maxDepth - 1; d > maxDepth - 5; d = d - 1)
                {
                    node = octree_->search(key, d);
                    if (node) 
                    {
                        break;
                    }
                }
            }
            if (node)
            {
                max_z = std::max(max_z, node->getOccupancy());
            }
        }
    }
    if (max_z > -std::numeric_limits<double>::infinity())
    {
        return max_z;
    }
    else
    {
        return 0.0;
    }

}


void Astar::cost_from_parent(const GridNode &parent, const GridNode &node)
{
    /* Returns the cost from the parent node to the current node */
    // Euclidean distance
    double dx = node.pose.x - parent.pose.x;
    double dy = node.pose.y - parent.pose.y;

    // Height difference
    height_diff = std::abs(find_node_height(node) - find_node_height(parent));

    return sqrt(dx * dx + dy * dy) + height_diff;
}

double Astar::h(const GridNode &node)
{
    /* Heuristic function for A* algorithm */
    // Euclidean distance
    double dx = node.pose.x - goal.x;
    double dy = node.pose.y - goal.y;

    return sqrt(dx * dx + dy * dy);
}

void Astar::create_node(const Pose2D &pose, GridNode &node)
{
    /* Creates a node from the given pose */
    node.pose = pose;

    if node.parent != nullptr
    {
        node.g = node.parent->g + cost_from_parent(node.parent, node);
    }
    else
    {
        node.g = 0.0;
    }
    node.f = node.g + h(node);
    node.parent = nullptr;
}

bool Astar::goal_reached(GridNode &current_node, GridNode &goal_node)
{
    /* Checks if the goal is reached */
    // Check if the goal is reached
    double dx = goal_node.pose.x - current_node.pose.x;
    double dy = goal_node.pose.y - current_node.pose.y;
    double dist = sqrt(pow(dx, 2) + pow(dy, 2));
    if (dist < xy_goal_tolerance)
    {
        return true;
    }
    return false;
}


void Astar::plan()
{
    /* Combines other parts of the code and runs planning */
    // makecostgrid();
    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
        ROS_INFO("Planning...");
        if (!goal_received) 
        {
            ROS_WARN_THROTTLE(5.0, "Waiting for goal...");
            loop_rate.sleep();
            continue;
        }
        if (!octree_) 
        {
            ROS_WARN_THROTTLE(5.0, "Waiting for octomap...");
            loop_rate.sleep();
            continue;
        }
        // Astar algorithm
        std::vector<GridNode> path;


        // 1. define start position
        GridNode start_node;
        create_node(current_pose, start_node);


        // 2. define goal position
        GridNode goal_node;
        create_node(goal, goal_node);
        // 3. create open and closed lists
        std::priority_queue<GridNode> open_list; // Uses the existing operator< for comparison
        std::vector<GridNode> closed_list;
        open_list.push_back(start_node);
        
        while (open_list.size() > 0)
        {
            // 4. pop the node with the lowest f value
            GridNode current_node = open_list.top();
            open_list.pop();

            // 5. check if goal is reached
            if (goal_reached(current_node, goal_node))
            {
                ROS_INFO("Goal found");
                // reconstruct path
                GridNode *node = &current_node;
                while (node != nullptr)
                {
                    path.push_back(*node);
                    node = node->parent;
                }
                std::reverse(path.begin(), path.end());
                break;
            }

            // 6. check if node is already in closed list and add if not
            if (closed_list.end() != std::find(closed_list.begin(), closed_list.end(), current_node))
            {
                continue; // already evaluated
            }
            closed_list.push_back(current_node);

            // 7. generate children nodes
            std::vector<GridNode> children_nodes;
            for (int i = -1; i <= 1; i++)
            {
                for (int j = -1; j <= 1; j++)
                {
                    if (i == 0 && j == 0) continue; // skip the parent node
                    Pose2D child_pose;

                    // TO DO: change robot_grid_n to a different resolution param
                    child_pose.x = current_node.pose.x + i * rf.robot_grid_n;
                    child_pose.y = current_node.pose.y + j * rf.robot_grid_n;
                    create_node(child_pose, child_node);
                    children_nodes.push_back(child_node);
                }
            }

            // 8. check for collisions and add to open list
            // for now pushing all nodes to open list (collision cost is already included in g)
            for (const auto &child : children_nodes)
            {
                // if (!check_collision(child))
                // {
                //     open_list.push(child);
                // }

                open_list.push(child);
            }
        }

        if (path.size() > 0)
        {
            ROS_INFO("Path found");
            // publish path
            vis.publishPath(path);
        }
        else
        {
            ROS_WARN("No path found");
        }
    }
}

bool Astar::check_collision(const std::vector<Pose2D> &traj)
{
    /* Checks if the current trajectory is collision-free 
    Returns true if there is a collision 
    Also calculates the height cost, if an obstacle exists
    */

    // // Just for debugging
    // int a = 0;
    // int b = 0;
    // int c = 0;

    bool obstacle_found = false;
    this->height_cost = 0.0; // Reset the height cost for every trajectory

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

        // Max height of the obstacle in the cells of the robot footprint
        double local_max_z = -std::numeric_limits<double>::infinity();  
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
                    obstacle_found = true;

                    if (z >= local_max_z)
                    {
                        local_max_z = z;
                    }
                }
            }
        }

        // If there is an obstacle in the cell, add the height of the obstacle to the height cost
        // For every pose in the trajectory, add the height of the highest obstacle in the robot footprint
        if (local_max_z > -std::numeric_limits<double>::infinity())
        {
            // ROS_INFO("Local max z: %f", local_max_z);
            // Absolute value because that will consider obstacles/ground the slopes both up and down
            this->height_cost += std::abs(local_max_z);
        }

    }

    return obstacle_found;
}

std::vector<std::pair<double, double>> Astar::transform_robot_footprint(const Pose2D &pose, std::vector<std::pair<double, double>> &robot_footprint)
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

std::vector<std::pair<double, double>> Astar::get_robot_grid(std::vector<std::pair<double, double>> &robot_footprint)
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


void Astar::get_octomap_bounds(octomap::OcTree* octree)
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

}// namespace astar
// Namespace is closed here before main because main is the entry point of the program
// compiler will fail to compile if the namespace is not closed before main
// main is supposed to be in global namespace


int main(int argc, char **argv)
{
    ros::init(argc, argv, "astar");

    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    astar::Astar planner(nh, &tfBuffer);
    planner.plan();
    return 0;
}