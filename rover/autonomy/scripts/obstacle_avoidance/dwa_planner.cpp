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

struct Pose2D {
  double x;
  double y;
  double theta;
};

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
    bool check_collision();

private:

    // Structs
    VelocityRange vr;
    DWAConfig dwac;
    double current_v;
    double current_w;
    Pose2D current_pose;
    
    // ROS communication
    tf2_ros::Buffer* tfBuffer_;
    ros::Subscriber octomap_sub;
    ros::Subscriber odom_sub;
    ros::Publisher cmd_vel_pub;
    std::shared_ptr<octomap::OcTree> octree_;
    std::string octomap_topic;
    std::string odom_topic;
    int rate;
};

// Constructor

DWAPlanner::DWAPlanner(ros::NodeHandle &nh, tf2_ros::Buffer* tfBuffer)
{
    // Variables
    // Private node handle to get parameters from the launch file where the parameter file is loaded in the dwa_planner node
    ros::NodeHandle pnh("~");
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
        // ROS_INFO("Current linear velocity: %f", current_v);
        // ROS_INFO("Current angular velocity: %f", current_w);

        // 1. Compute feasible velocity ranges (dynamic window)
        double v_min = std::max(this->vr.v_min, current_v - this->dwac.max_accel_lin * this->dwac.dt);
        double v_max = std::min(this->vr.v_max, current_v + this->dwac.max_accel_lin * this->dwac.dt);
        double w_min = std::max(this->vr.w_min, current_w - this->dwac.max_accel_ang * this->dwac.dt);
        double w_max = std::min(this->vr.w_max, current_w + this->dwac.max_accel_ang * this->dwac.dt);

        // 2. Discretize velocity space
        double best_cost = std::numeric_limits<double>::infinity();
        double best_v = 0.0;
        double best_w = 0.0;

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

                // For every velocity, simulate the trajectory for next sim_time sec and check for collision
                // The trajectory has the same velocity for the entire sim_time
                // Find the trajectory with the least cost
                for (double t = 0; t < this->dwac.sim_time; t += this->dwac.dt)
                {
                    // 4.1 Update pose
                    updated_pose.x += v * cos(current_pose.theta) * this->dwac.dt;
                    updated_pose.y += v * sin(current_pose.theta) * this->dwac.dt;
                    updated_pose.theta += w * this->dwac.dt;



                    // 4.2 Check for collision
                    if (!check_collision())
                    {
                        break;
                    }
                }
                // 5. Compute cost
                // 6. Update best trajectory
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
        
    }

}

bool DWAPlanner::check_collision()
{
    /* Checks if the current trajectory is collision-free */

    return true;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "dwa_planner");

    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    DWAPlanner planner(nh, &tfBuffer);
    planner.plan();
    ros::spin();
    return 0;
}