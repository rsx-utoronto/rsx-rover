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


class TeleopRover
{
public:
	TeleopRover();
	void publishDrive();
	void pubConstSpeed();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
	void networkCallback(const std_msgs::Bool::ConstPtr& net_stat);

	ros::NodeHandle nh;
	double robot_radius = 0.8;
	double MAX_LINEAR_SPEED = 2.5;
	double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED*robot_radius;
	double gear = 0; // set to 0 initially
	ros::Publisher drive_pub_;
	ros::Subscriber joy_sub;
	ros::Subscriber net_sub;
	int network_status = 1;
	geometry_msgs::Twist twist;
};

TeleopRover::TeleopRover()
{
	TeleopRover::network_status = false;
	drive_pub_ = nh.advertise<geometry_msgs::Twist>("drive", 1);
	TeleopRover::joy_sub = nh.subscribe("joy", 10, &TeleopRover::joyCallback, this);
	TeleopRover::net_sub = nh.subscribe("network_status", 1, &TeleopRover::networkCallback, this);
}

void TeleopRover::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{

	// indexs for controller values
	int R2 = 5;
	int L2 = 2;
	int LS_x = 0; // x axis of joystick
	int LS_y = 1; // y axis of joystick
	int dec_speed = 4;
	int inc_speed = 5;

	// Values from Controller
	double posThrottle = joy->axes[R2];
	double negThrottle = joy->axes[L2];
	double turnFactor_x = static_cast<double>(joy->axes[LS_x]);
	double turnFactor_y = static_cast<double>(joy->axes[LS_y]);
	double lin_vel;

	// Encoding value from joystick 

	if (turnFactor_y != 0) {
		lin_vel = 255.0 * turnFactor_y;
	} else {
		lin_vel = 0;
	}

	// Encoding values for gear selection (range 0 - 1)
	if (joy->buttons[dec_speed] == 1) // reduce
	{
		if (gear >= 0.1)
			gear -= 0.1;
		else
			gear = 0;
	}
	else if (joy->buttons[inc_speed] == 1) // increase
	{
		if (gear <= 0.9)
			gear += 0.1;
		else
			gear = 1;
	}

	lin_vel = lin_vel * gear;
	ROS_INFO("Linear velocity: %f", lin_vel);
	twist.linear.x = static_cast<double>(lin_vel/(double)255.0)*MAX_LINEAR_SPEED; // Should be in range of -MAX_LINEAR_SPEED to +MAX_LINEAR_SPEED 
	twist.angular.z = static_cast<double>(turnFactor_x)*MAX_ANGULAR_SPEED; // Should be in range of -MAX_ANGULAR_SPEED to +MAX_ANGULAR_SPEED 

	ROS_INFO("Turn Factor X %f", turnFactor_x);
	ROS_INFO("Turn Factor Y %f", turnFactor_y);
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

void TeleopRover::pubConstSpeed() {
	geometry_msgs::Twist test_msg;
	test_msg.linear.x = 1.0;
	drive_pub_.publish(test_msg);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "drive_sender_falcon");
	TeleopRover drive_sender;
	ros::spin();
}
