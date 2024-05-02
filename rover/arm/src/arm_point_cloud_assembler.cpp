#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

class PointCloudAssembler{
  private:
    ros::Timer fiveSecTimer;
    ros::NodeHandle node;
    ros::Subscriber pointCloudInSub;
    ros::Publisher pointCloudOutPub;

    pcl::PCLPointCloud2 currentPointCloud;

    bool isFirstTime = false;
    bool isAllowedToUpdate = true;
    
  public: 
    PointCloudAssembler(){
      pointCloudOutPub = node.advertise<sensor_msgs::PointCloud2>("/arm/assembled_point_cloud", 1);
      pointCloudInSub = node.subscribe("/cloud_input", 1, &PointCloudAssembler::onCloudInput, this);

      fiveSecTimer = node.createTimer(ros::Duration(2,0), &PointCloudAssembler::fiveSecTimerCallback, this);
    }

    void fiveSecTimerCallback(const ros::TimerEvent& e){
      isAllowedToUpdate = true;
    }

    void onCloudInput(const sensor_msgs::PointCloud2ConstPtr& input){
      pcl::PCLPointCloud2* cloudIn = new pcl::PCLPointCloud2;
      pcl_conversions::toPCL(*input, *cloudIn);
      ROS_INFO("input recieved");
      if(isAllowedToUpdate){
        if(!isFirstTime){
          currentPointCloud = *cloudIn;
          isFirstTime = true;
          isAllowedToUpdate = false;
          return ;
        }
        ROS_INFO("I WAS ALLOWED TO UPDATE");
        pcl::PCLPointCloud2 cloudOut = currentPointCloud + *cloudIn;
        // pcl::concatenateFields(currentPointCloud, *cloudIn, cloudOut);

        sensor_msgs::PointCloud2 output;
        pcl_conversions::fromPCL(cloudOut, output);

        pointCloudOutPub.publish(output);
        currentPointCloud = cloudOut;

        isAllowedToUpdate = false;
      }
    }
};

int main (int argc, char** argv) {
  ros::init(argc, argv, "arm_point_cloud_assembler"); // start node
  ROS_INFO("node started");
  PointCloudAssembler pointCloudAssembler;

  // Spin
  ros::spin ();
}
