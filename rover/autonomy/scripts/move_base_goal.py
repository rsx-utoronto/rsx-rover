#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class MoveBaseGoalSet():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()

        self.goal = MoveBaseGoal()

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


        self.client.send_goal(self.goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            self.client.get_result()
            rospy.loginfo("Goal execution done!")
if __name__ == '__main__':
    rospy.init_node('move_base_goal', anonymous=True)

    move_base_goal = MoveBaseGoalSet()
    move_base_goal.send_goal(1, 1)