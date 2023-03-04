#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <boost/thread.hpp>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace message_filters;

class TeleopRover {
	public:
		TeleopRover();
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		void networkCallback(const std_msgs::Bool::ConstPtr& net_stat);

		ros::NodeHandle nh;
		float MAX_ANGULAR_SPEED = 0.4;
		float MAX_LINEAR_SPEED = 0.6;
		ros::Publisher drive_pub_;
		ros::Subscriber joy_sub;
		ros::Subscriber net_sub;
		geometry_msgs::Twist twist;
		bool network_status;
};

TeleopRover::TeleopRover()
{
	TeleopRover::network_status = false;
	drive_pub_ = nh.advertise<geometry_msgs::Twist>("drive", 1);
	TeleopRover::joy_sub = nh.subscribe("joy", 10, &TeleopRover::joyCallback, this);
	TeleopRover::net_sub = nh.subscribe("network_status", 1, &TeleopRover::networkCallback, this);
}

void TeleopRover::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

	//indexs for controller values
	int R2 = 5;
	int L2 = 2;
	int LS = 0;

	//Values from Controller
	double posThrottle = joy->axes[R2];
	double negThrottle = joy->axes[L2];
	float turnFactor = static_cast<float>(joy->axes[LS]);
	double lin_vel;

	double dispVal = 0;

	//Encoding Values for Throttle
	if (posThrottle < 1 && negThrottle < 1){
		dispVal = 0;
		lin_vel =  0;

	} else if (posThrottle < 1){
		ROS_INFO("in Pos throttle");
		dispVal = 255 - (posThrottle+1)*127.5;
		lin_vel = 255 - (posThrottle+1)*127.5;
	} else if (negThrottle < 1){
		ROS_INFO("in neg throttle");
		dispVal = -1*(255 - (negThrottle+1)*127.5);
		lin_vel = -1*(255 - (negThrottle+1)*127.5);
	} else {
		dispVal = 0;
		lin_vel = 0;
	}

	twist.linear.x = lin_vel/255*MAX_LINEAR_SPEED;
	twist.angular.z = turnFactor/1*MAX_ANGULAR_SPEED;

	ROS_INFO("Turn Factor %f", turnFactor);
	ROS_INFO("Motor Value %f", lin_vel);

	drive_pub_.publish(twist);
}

void TeleopRover::networkCallback(const std_msgs::Bool::ConstPtr& net_stat){
	if (net_stat->data == false){
		TeleopRover::network_status = false;
		twist.linear.x = 0; 
		twist.linear.y = 0;
		twist.linear.z = 0;
		twist.angular.x = 0;
		twist.angular.y = 0;
		twist.angular.z = 0;
		drive_pub_.publish(twist);
	} else {
		TeleopRover::network_status = true;
	}
	
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "drive_sender_falcon");
	TeleopRover drive_sender;
	ros::spin();
}
