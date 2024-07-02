#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <rover/StateMsg.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <boost/thread.hpp>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cmath>

/*
- R2 to accelerate, L2 to decelarate 
	- If both, move at that constant speed
- Triangle to increase gear, square to decrease gear
- Left joystick to control steering
	- the y vector is not used
	- the x vector is used to: 
		- control steering (more x, more steering)
		- control lnear speed (more x, less lin speed)
- Circle to turn completely on spot (linear speed = 0)
*/

class TeleopRover
{
public:
	TeleopRover(); // constructor (just like __init__ in python) // has to be the same name as the class
	void publishDrive();
	void pubConstSpeed();
	void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
	void networkCallback(const std_msgs::Bool::ConstPtr& net_stat);
	void stateCallback(const rover::StateMsg::ConstPtr& state);
	void SetVelocity();

	ros::NodeHandle nh;
	double robot_radius = 1;
	double MAX_LINEAR_SPEED = 2.5; // 2.5 speed est * 0.65 from rough calibration
	double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED*robot_radius;
	int RATE = 20;
	double MAX_ACCELERATION = 0.3/RATE; // 0.1 m/s^2
	double lin_vel = 0;
	double prev_lin_vel = 0;
	double ang_vel = 0;
	// double TIME = 
	double gear = 0; // set to 0 initially
	ros::Publisher drive_pub;
	ros::Subscriber joy_sub;
	ros::Subscriber net_sub;
	ros::Subscriber state_sub;
	bool network_status = false;
	geometry_msgs::Twist twist;
	bool MANUAL_ENABLED = true;
	bool KILL_PRESSED = false;
	bool gear_pressed = false;
	// Initialize buttons and axes with default values
	// this gets run every time the loop in SetVelocity runs
	// This is fine in general except when we need to remember the previous value of the buttons (like in the case of KILL)
	std::vector<int> buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	std::vector<float> axes = {-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0};
};

TeleopRover::TeleopRover()
{
	drive_pub = nh.advertise<geometry_msgs::Twist>("drive", 1);
	TeleopRover::joy_sub = nh.subscribe("/software/joy", 10, &TeleopRover::joyCallback, this);
	TeleopRover::net_sub = nh.subscribe("/network_status", 1, &TeleopRover::networkCallback, this);
	TeleopRover::state_sub = nh.subscribe("/rover_state", 1, &TeleopRover::stateCallback, this);
	// network_status = false;
}

void TeleopRover::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
	buttons = joy->buttons;
	axes = joy->axes;
	// ROS_INFO("Joy Callback %f", axes[0]);
	
}

void TeleopRover::networkCallback(const std_msgs::Bool::ConstPtr& net_stat){
	network_status = net_stat->data;
}

void TeleopRover::stateCallback(const rover::StateMsg::ConstPtr& state_msg){
	MANUAL_ENABLED = state_msg->MANUAL_ENABLED;
}

void TeleopRover::SetVelocity(){
	ros::Rate loop_rate(RATE);
	while (ros::ok())
	{
		int KILL = 10; // PS button to kill (0 velocity) as soon as it is pressed
		if (buttons[KILL] == 1){
			ROS_INFO("KILL ENGAGED\n\n\n\n\n\n\n\n\n\n");
			twist.linear.x = 0;
			twist.linear.y = 0;
			twist.linear.z = 0;
			twist.angular.x = 0;
			twist.angular.y = 0;
			twist.angular.z = 0;
			drive_pub.publish(twist);
			prev_lin_vel = 0;	
			KILL_PRESSED = true;
		}
		else {
				ROS_INFO("KILL DISENGAGED");
				KILL_PRESSED = false;
			}

		if (MANUAL_ENABLED && ~KILL_PRESSED){
			twist.linear.x = 0; 
			twist.linear.y = 0;
			twist.linear.z = 0;
			twist.angular.x = 0;
			twist.angular.y = 0;
			twist.angular.z = 0;
			// indexs for controller values
			int R2 = 5;
			int L2 = 2;
			int LS_x = 0; // x axis of left joystick
			int LS_y = 1; // y axis of left joystick
			int L1 = 4; // L1
			int R1 = 5; // R1
			int X = 0; // CROSS button
			int C = 1; // CIRCLE button
			int T = 2; // TRIANGLE button
			int S = 3; // SQUARE button

			// Values from Controller
			double posThrottle = static_cast<double>(axes[R2]);
			double negThrottle = static_cast<double>(axes[L2]);
			double turnFactor_x = static_cast<double>(axes[LS_x]);
			double turnFactor_y = static_cast<double>(axes[LS_y]);
			ROS_INFO("LS_x:%f", turnFactor_x);
			ROS_INFO("LS_y:%f", turnFactor_y);

			posThrottle = (posThrottle + 1) / 2; // Normalizing values from 0 to 1
			negThrottle = (negThrottle + 1) / 2; // Normalizing values from 0 to 1
			posThrottle = 1 - posThrottle; // Inverting values because posThrottle is 1 when not pressed, 0 when completely pressed
			negThrottle = 1 - negThrottle; // Inverting values
			// ROS_INFO("R2:%f", posThrottle);
			// ROS_INFO("L2:%f", negThrottle);

			double max_allowed_lin_speed = gear * MAX_LINEAR_SPEED; // Maximum speed allowed for the gear
			double acc = 0; // Acceleration value based on how much R2 is pressed
			double max_allowed_ang_speed = gear * MAX_ANGULAR_SPEED; // Maximum speed allowed for the gear

			if (gear_pressed)
			{
				if (buttons[T] != 1 && buttons[X] != 1)
				{
					gear_pressed = false;
				}
			}
			else
			{
				// Encoding values for gear selection (range 0 - 1)
				if (buttons[X] == 1) // decrease
				{
					if (gear >= 0.1) // Change this value to change the gear step
						gear -= 0.1;
					else
						gear = 0;
					gear_pressed = true;
				}
				else if (buttons[T] == 1) // increase
				{
					if (gear <= 0.9)
						gear += 0.1;
					else
						gear = 1;
					gear_pressed = true;
				}
			}
			ROS_INFO("KILL_ENGAGED: %d", KILL_PRESSED);
			// This is a bit repetitive (there was an if KILL_PRESSED earlier) but it works so I am not gonna change it
			if (KILL_PRESSED == true)
			{
				lin_vel = 0;
				prev_lin_vel = 0;
			}
			else if (buttons[C] == 1)
			{
				// When Circle is pressed, turns on spot with 0 linear speed
				// Turns the rover in place clockwise with max angular speed for that gear
				lin_vel = 0;
				prev_lin_vel = 0;
				ang_vel = -max_allowed_ang_speed;
			}
			else if (buttons[S] == 1)
			{
				// When X is pressed, turns on spot with 0 linear speed
				// Turns the rover in place counter-clockwise with max angular speed for that gear
				lin_vel = 0;
				prev_lin_vel = 0;
				ang_vel = max_allowed_ang_speed;
			}
			else if (posThrottle != 0 && negThrottle != 0)
			{
				lin_vel = prev_lin_vel;
				ROS_INFO("Both Throttle Pressed %f", lin_vel);
			}
			else if (posThrottle !=0)
			{
				acc = posThrottle * MAX_ACCELERATION;
				if ((lin_vel + acc) < max_allowed_lin_speed)
				{
					// ROS_INFO("acc = %f", acc);
					// ROS_INFO("lin_vel = %f", lin_vel);
					lin_vel = prev_lin_vel + acc;
					prev_lin_vel = lin_vel;
				}
				else
				{
					if (prev_lin_vel <= max_allowed_lin_speed)
					{
						lin_vel = max_allowed_lin_speed;
						prev_lin_vel = lin_vel;
					}
					else
					{
						// ROS_INFO("Gear reduced\n\n\n");
						lin_vel = prev_lin_vel - MAX_ACCELERATION;
						// ROS_INFO("lin_vel = %f", lin_vel);
						prev_lin_vel = lin_vel;
					}
				}
				ROS_INFO("Positive Throttle Pressed %f", lin_vel);
			}
			else if (negThrottle != 0)
			{
				acc = negThrottle * MAX_ACCELERATION;
				// ROS_INFO("lin_vel = %f", std::fabs(lin_vel - acc));
				if ((std::fabs(lin_vel - acc)) < max_allowed_lin_speed) // floating point aboslute value - fabs
				{
					lin_vel = prev_lin_vel - acc;
					prev_lin_vel = lin_vel;
				}
				else
				{
					if (std::fabs(prev_lin_vel) <= max_allowed_lin_speed)
					{
						lin_vel = -max_allowed_lin_speed;
						prev_lin_vel = lin_vel;
					}
					else
					{
						// This is to prevent from going to infinity when the gear is decreased to 0 and the prev_lin_vel becomes greater than 0 (when we do +MAX_ACCELERATION)
						if (lin_vel < 0)
						{
							lin_vel = prev_lin_vel + MAX_ACCELERATION;
							prev_lin_vel = lin_vel;
						}
						else
						{
							lin_vel = 0;
							prev_lin_vel = lin_vel;
						}
					}
				}
				ROS_INFO("Negative Throttle Pressed %f", lin_vel);
			}
			else
			{
				if (prev_lin_vel != 0) // This is equal to 0 when the throttle is not pressed in the beginning or the kill is engaged
				{
					if (lin_vel > 0)
					{
						lin_vel = prev_lin_vel - MAX_ACCELERATION;
						prev_lin_vel = lin_vel;
						if (lin_vel < 0)
						{
							lin_vel = 0;
							prev_lin_vel = 0;
						}
					}
					else
					{
						lin_vel = lin_vel + MAX_ACCELERATION;
						prev_lin_vel = lin_vel;
						if (lin_vel > 0)
						{
							lin_vel = 0;
							prev_lin_vel = 0;
						}
					}
				}
				else
				{
					lin_vel = 0;
					prev_lin_vel = 0;
				}
				
				ROS_INFO("No Throttle Pressed %f", lin_vel);
			}
			
			ROS_INFO("Linear velocity: %f", (lin_vel));
			ROS_INFO("Angular velocity: %f", (ang_vel));
			twist.linear.x = static_cast<double>(lin_vel); // Should be in range of -MAX_LINEAR_SPEED to +MAX_LINEAR_SPEED 
			twist.angular.z = static_cast<double>(ang_vel); // Should be in range of -MAX_ANGULAR_SPEED to +MAX_ANGULAR_SPEED 

			// ROS_INFO("Turn Factor X %f", turnFactor_x);
			// ROS_INFO("Turn Factor Y %f", turnFactor_y);
			// ROS_INFO("Motor Value %f", lin_vel);
			ROS_INFO("GEAR %f", gear);

			drive_pub.publish(twist);
		}
		ros::spinOnce(); // for some reason, without this line, the code just remains in this loop and doesn't exit which doesn't let the subscriber to get data
		loop_rate.sleep();
	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "drive_sender_node");
	TeleopRover drive_sender;
	drive_sender.SetVelocity();
	// ros::spin(); // no need for this since we have a while loop in SetVelocity function
}
