#include "ros/ros.h"
#include "std_msgs/String.h"

void writeMsgToLog(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("The message that we received was: %s", msg->data.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Subscriber");
    ros::NodeHandle n;

    ros::Subscriber topic_sub = n.subscribe("topic_name", 1000, writeMsgToLog);

    ros::spin();

    return 0;
}