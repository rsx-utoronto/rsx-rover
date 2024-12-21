#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32MultiArray
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def talker():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_joint_angle_publisher', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "rsx_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    pub = rospy.Publisher('arm_goal_pos', Float32MultiArray, queue_size=10)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        joint_goal = move_group.get_current_joint_values()
        joint_goal_msg = Float32MultiArray()
        joint_goal_msg.data = joint_goal
        pub.publish(joint_goal_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass