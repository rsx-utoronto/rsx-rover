
#include "ros_phoenix/ArmVelocities.h"
#include "ros_phoenix/MotorStatus.h"

#include <string>
#include <ros/ros.h>


ros::NodeHandle n("~");
std::string motor_name;

void statusCallbackLeft(const ros_phoenix::MotorStatus msg)
{
    ros::Publisher positionPub = n.advertise<std_msgs::Float32>(motor_name+"/position", 1000);
    if(ros::ok()){

        std_msgs::Float32 position;
        position.data = msg.position
        positionPub.publish(position);
    }
}


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "wheel_nodes");

  n.getParam("motor_name", motor_name);

  ros::Subscriber sub = n.subscribe(motor_name+"/status", 1000, statusCallback);
  
  ros::Rate loop_rate(10);

  return 0;
}

