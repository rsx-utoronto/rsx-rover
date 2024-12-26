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
  double w_dist;
  double w_vel;
};


class DWAPlanner
{
public:
    DWAPlanner(ros::NodeHandle &nh);
    
    // Parameters
    void define_parameters(ros::NodeHandle &nh);

    // Callbacks
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg);
    // void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);

    // Planning
    void plan();

private:

    // Structs
    VelocityRange vr;
    DWAConfig dwac;
    
    // ROS communication
    ros::Subscriber octomap_sub;
    // ros::Subscriber cmd_vel_sub;
    ros::Publisher cmd_vel_pub;
    std::shared_ptr<octomap::OcTree> octree_;
    std::string octomap_topic;
};

// Constructor

DWAPlanner::DWAPlanner(ros::NodeHandle &nh)
{
    // Variables
    // Private node handle to get parameters from the launch file where the parameter file is loaded in the dwa_planner node
    ros::NodeHandle pnh("~");
    DWAPlanner::define_parameters(pnh);
    
    octomap_sub = nh.subscribe(octomap_topic, 1, &DWAPlanner::octomapCallback, this);
    // cmd_vel_sub = nh.subscribe("/cmd_vel", 1, &DWAPlanner::cmdVelCallback, this);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

// Parameters

void DWAPlanner::define_parameters(ros::NodeHandle &nh)
{
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

    // Topics
    nh.param<std::string>("octomap_topic", octomap_topic, "/octomap_full");
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



void DWAPlanner::plan()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dwa_planner");

    ros::NodeHandle nh;
    DWAPlanner planner(nh);
    ros::spin();
    return 0;
}