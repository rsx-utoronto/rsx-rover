#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char** argv)
{
    // Initialize ROS and create a node handle
    ros::init(argc, argv, "publish_pcd");
    ros::NodeHandle nh;

    // Create a ROS publisher
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("point_cloud", 1);

    // Load the PCD file
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/arwin/rover_ws/src/rsx-rover/rover/autonomy/tests/pcd/combined.pcd", cloud) == -1)
    {
        PCL_ERROR ("Couldn't read the pcd file \n");
        return (-1);
    }

    // Convert the PCL point cloud to a ROS message
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(cloud, ros_cloud);

    // Set the frame ID
    ros_cloud.header.frame_id = "map";

    // Create a loop rate object
    ros::Rate loop_rate(10);

    // Publish the point cloud until the node is stopped
    while (ros::ok())
    {
        // Update the timestamp and publish the message
        ros_cloud.header.stamp = ros::Time::now();
        pub.publish(ros_cloud);

        // Spin and sleep for the remainder of the loop
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
