#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>

const float WHEEL_CIRMUFERENCE = 100; //need to set correct value
const int TICKS_PER_REV = 2048;
const float DIST_WHEEL_CENTER = 350; //need to set correct value

float prevEncoderTickLeft;
float prevEncoderTickRight;

float distanceLeftWheel;
float distanceRightWheel;


ros::NodeHandle n;

void updateInfo(){
  std_msgs::Float32 theta;
  std_msgs::Float32 totalDistance;
  std_msgs::Float32 positionX;
  std_msgs::Float32 positionY;

  theta.data = (distanceRightWheel - distanceLeftWheel)/(2*DIST_WHEEL_CENTER);
  totalDistance.data = (distanceLeftWheel + distanceRightWheel)/2;
  positionX.data = totalDistance.data * cos(theta.data);
  positionY.data = totalDistance.data * sin(theta.data);


  ros::Publisher thetaPub = n.advertise<std_msgs::Float32>("odometry_theta", 1000);
  ros::Publisher distancePub = n.advertise<std_msgs::Float32>("total_distance", 1000);
  ros::Publisher positionXPub = n.advertise<std_msgs::Float32>("position_x", 1000);
  ros::Publisher positionYPub = n.advertise<std_msgs::Float32>("position_y", 1000);

  if (ros::ok()){
    thetaPub.publish(theta);
    distancePub.publish(totalDistance);
    positionXPub.publish(positionX);
    positionYPub.publish(positionY);
  }

}


void statusLeftCallback(const std_msgs::Float32 msg)
{
  float leftPos = msg.data;
  distanceLeftWheel = (leftPos - prevEncoderTickLeft)*WHEEL_CIRMUFERENCE/TICKS_PER_REV;
  prevEncoderTickLeft = leftPos;
  updateInfo();
}

void statusRightCallback(const std_msgs::Float32 msg){
  float rightPos = msg.data;
  distanceRightWheel = (rightPos - prevEncoderTickRight)*WHEEL_CIRMUFERENCE/TICKS_PER_REV;
  prevEncoderTickRight = rightPos;
  updateInfo();
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "wheel_odometry");

  ros::Subscriber leftWheelSub = n.subscribe("mid_left/status", 1000, statusLeftCallback);
  ros::Subscriber RightWheelSub = n.subscribe("mid_right/status", 1000, statusRightCallback);

  ros::spin();

  return 0;
}