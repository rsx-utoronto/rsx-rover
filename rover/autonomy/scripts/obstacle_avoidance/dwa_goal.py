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
        self.rate = rospy.Rate(0.5)

    def send_goal(self, pos_x = 0, pos_y = 0, pos_z = 0, orient_x = 0, orient_y = 0, orient_z = 0, orient_w = 1):
        
        while not rospy.is_shutdown():
            rospy.loginfo("Sending goal")

            self.goal.header.stamp = rospy.Time.now()
            self.goal.header.frame_id = 'map'
            self.goal.pose.position.x = pos_x
            self.goal.pose.position.y = pos_y
            self.goal.pose.position.z = pos_z
            self.goal.pose.orientation.x = orient_x
            self.goal.pose.orientation.y = orient_y
            self.goal.pose.orientation.z = orient_z
            self.goal.pose.orientation.w = orient_w


            self.goal_pub.publish(self.goal)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('dwa_goal', anonymous=True)

    dwa_goal = DWAGoalSet()
    goal_x = rospy.get_param('~goal_x', 0)
    goal_y = rospy.get_param('~goal_y', 0)
    dwa_goal.send_goal(goal_x, goal_y)