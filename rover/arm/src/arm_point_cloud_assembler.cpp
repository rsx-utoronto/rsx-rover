#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>

class PointCloudAssembler{
	private:
		ros::Timer fiveSecTimer;
		ros::NodeHandle node;
		ros::Subscriber pointCloudInSub;
		ros::Publisher pointCloudOutPub;
		ros::Publisher otherCloudPub;
		ros::Publisher filteredCloudPub;

		// pcl::PCLPointCloud2 currentPointCloud;
		pcl::PointCloud<pcl::PointXYZ> currentPointCloud;
		sensor_msgs::PointCloud2 currentPCMes;

		bool isFirstTime = true;
		bool isAllowedToUpdate = true;
    
	public: 
		PointCloudAssembler(){
			pointCloudOutPub = node.advertise<sensor_msgs::PointCloud2>("/arm/assembled_point_cloud", 1);
			otherCloudPub = node.advertise<sensor_msgs::PointCloud2>("/arm/other_cloud", 1);
			filteredCloudPub = node.advertise<sensor_msgs::PointCloud2>("/arm/filtered_cloud", 1);
			pointCloudInSub = node.subscribe("/cloud_input", 1, &PointCloudAssembler::onCloudInput, this);

			fiveSecTimer = node.createTimer(ros::Duration(2,0), &PointCloudAssembler::fiveSecTimerCallback, this);
		}

			void fiveSecTimerCallback(const ros::TimerEvent& e){
			isAllowedToUpdate = true;
		}

   		void onCloudInput(const sensor_msgs::PointCloud2ConstPtr& input){
			// pcl::PCLPointCloud2 cloudIn;
			// pcl_conversions::toPCL(*input, cloudIn);
			pcl::PointCloud<pcl::PointXYZ>::Ptr pclInPtr(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromROSMsg(*input, *pclInPtr);

			pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
			voxelFilter.setInputCloud(pclInPtr);
			voxelFilter.setLeafSize(0.1, 0.1, 0.1);
			voxelFilter.filter(*filteredCloud);

			if(isFirstTime){
				currentPointCloud = *filteredCloud;
			  	isFirstTime = false;
				ROS_INFO("First time!");
			}
			else currentPointCloud += *filteredCloud;

			sensor_msgs::PointCloud2 output, filteredOutput;
			pcl::toROSMsg(currentPointCloud, output);
			pcl::toROSMsg(*filteredCloud, filteredOutput);
			// output.header = input->header;
			pointCloudOutPub.publish(output);
			filteredCloudPub.publish(filteredOutput);

			// ROS_INFO("I WAS ALLOWED TO UPDATE");
		}

		void combinePointCloud(const sensor_msgs::PointCloud2Ptr& pc2){
			if(isAllowedToUpdate){
				if(!currentPCMes.data.empty()){
					currentPCMes.data.insert(currentPCMes.data.end(), pc2->data.begin(), pc2->data.end());
					currentPCMes.width += pc2->width;
				}
				else{
					currentPCMes = *pc2;
				}

				pointCloudOutPub.publish(currentPCMes);
			}
		}
};

int main (int argc, char** argv) {
	ros::init(argc, argv, "arm_point_cloud_assembler"); // start node
	ROS_INFO("node started");
	PointCloudAssembler pointCloudAssembler;

	// Spin
	ros::spin ();
	return 0;
}
