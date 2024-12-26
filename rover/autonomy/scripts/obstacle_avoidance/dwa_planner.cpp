#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>


struct VelocityRange {
  double v_min;
  double v_max;
  double w_min;
  double w_max;
};

struct DWAConfig {
  double max_accel_lin;
  double max_accel_ang;
  double dt;        // time step for simulation
  double sim_time;  // horizon T
  int num_samples_v;
  int num_samples_w;

  // Cost function weights
  double w_heading;
  double w_distance;
  double w_velocity;
};


class DWAPlanner
{
public:
    DWAPlanner(ros::NodeHandle &nh, DWAConfig config);
    
    // Callbacks
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg);
    // void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);

private:

    // Variables

    // ROS communication
    ros::Subscriber octomap_sub;
    // ros::Subscriber cmd_vel_sub;
    ros::Publisher cmd_vel_pub;
    std::shared_ptr<octomap::OcTree> octree_;
};

// Constructor

DWAPlanner::DWAPlanner(ros::NodeHandle &nh, const DWAConfig config)
{
    octomap_sub = nh.subscribe("/octomap_full", 1, &DWAPlanner::octomapCallback, this);
    // cmd_vel_sub = nh.subscribe("/cmd_vel", 1, &DWAPlanner::cmdVelCallback, this);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

void DWAPlanner::octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
{
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
    // ROS_INFO("Resolution: %f", octree->getResolution());
    // ROS_INFO("Tree depth: %d", octree->getTreeDepth());
    // ROS_INFO("Tree value probability: %f", octree->getOccupancyThres());
    

    // Store it for later use in collision checks
    octree_.reset(octree);
    // return octree;
    // Do something with the octree
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dwa_planner");

    ros::NodeHandle nh;
    DWAConfig config;
    DWAPlanner planner(nh, config);
    ros::spin();
    return 0;
}