#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"

#define JOINT_NUM 1

class RSX_Arm: public hardware_interface::RobotHW 
{
    protected:
        hardware_interface::JointStateInterface joint_state_interface;
        hardware_interface::EffortJointInterface effort_joint_interface;
        hardware_interface::PositionJointInterface position_joint_interface;
        
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface;
        joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;
        
        double joint_position[JOINT_NUM];
        double joint_velocity[JOINT_NUM];
        double joint_effort[JOINT_NUM];
        double joint_effort_command[JOINT_NUM];
        double joint_position_command[JOINT_NUM];
        
        ros::NodeHandle nh;
        ros::Timer rosLoop;
        ros::Duration elapsed_time;
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

        ros::Subscriber goalPos;
        ros::Publisher currPos;

    public:
        void init(){
            const char* joints[JOINT_NUM] = {"joint_1"};

            hardware_interface::JointStateHandle stateHandel1(joints[0], &joint_position[0], 
                &joint_velocity[0], &joint_effort[0]);
            joint_state_interface.registerHandle(stateHandel1);

            registerInterface(&joint_state_interface);

            hardware_interface::JointHandle posHandle1(stateHandel1, &joint_position_command[0]);
            position_joint_interface.registerHandle(posHandle1);

            registerInterface(&joint_position_command);
            registerInterface(&position_joint_interface);

        }
        RSX_Arm(ros::NodeHandle& nh_): nh(nh_){
            init();
            controller_manager_.reset(new controller_manager::ControllerManager(this, nh));

            loop_hz_ = 5;

            currPos = nh.advertise<std_msgs::Float32MultiArray>("/arm_curr_pos", 1);
            goalPos = nh.subscribe("/arm_goal_pos", 1, &RSX_Arm::read, this);
            
            ros::Duration update_freq = ros::Duration(1.0/loop_hz_);

            rosLoop = nh.createTimer(update_freq, &RSX_Arm::update, this);
        }

        ~RSX_Arm(){

        }

        void update(const ros::TimerEvent& e){
            elapsed_time = ros::Duration(e.current_real - e.last_real);
            controller_manager_->update(ros::Time::now(), elapsed_time);
            RSX_Arm::write();
        }

        void read(const std_msgs::Float32MultiArray::ConstPtr& msg){
            ROS_INFO("Read %f", msg->data[0]);
            joint_position[0] = msg->data[0]; 
        }

        void write(){
            ROS_INFO("Write %f", joint_position_command[0]);
            std_msgs::Float32 msg;

            msg.data = joint_position_command[0];
            currPos.publish(msg);
            ROS_INFO("msg sent");
        }
        
};

int main(int argc, char** argv){
    ros::init(argc, argv, "rsx_arm_hardware_interface");

    ros::NodeHandle node;

    ROS_INFO("node started");
    RSX_Arm arm(node);
    ROS_INFO("node destroyed");

    ros::MultiThreadedSpinner spinner(2); // need with launch file
    spinner.spin();

    ROS_INFO("I died");
    return 0;
}