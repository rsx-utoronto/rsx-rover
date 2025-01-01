#!/usr/bin/env python3
"""
Gives goals to DWA local planner
"""
import rospy
from geometry_msgs.msg import PoseStamped

class DWAGoalSet():
    def __init__(self):
        self.goal_pub = rospy.Publisher(rospy.get_param('~goal_topic', "/goal"), PoseStamped, queue_size=10)
        self.goal = PoseStamped()

    def send_goal(self, pos_x = 0, pos_y = 0, pos_z = 0, orient_x = 0, orient_y = 0, orient_z = 0, orient_w = 1):
        rospy.loginfo("Sending goal")

        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = pos_x
        self.goal.target_pose.pose.position.y = pos_y
        self.goal.target_pose.pose.position.z = pos_z
        self.goal.target_pose.pose.orientation.x = orient_x
        self.goal.target_pose.pose.orientation.y = orient_y
        self.goal.target_pose.pose.orientation.z = orient_z
        self.goal.target_pose.pose.orientation.w = orient_w


        self.goal_pub.publish(self.goal)

if __name__ == '__main__':
    rospy.init_node('dwa_goal', anonymous=True)

    move_base_goal = DWAGoalSet()
    goal_x = rospy.get_param('~goal_x', 0)
    goal_y = rospy.get_param('~goal_y', 0)
    move_base_goal.send_goal(goal_x, goal_y)