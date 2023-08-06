#!/usr/bin/env python3

import rospy
import smach 
from smach_ros import SimpleActionState
from rover.msg import StateMsg
from nav_msgs.msg import Odometry
import numpy as np


class GPSCheckerNode():

    def __init__(self):
        # CHANGE
        self.curr_UTM_subscriber = rospy.Subscriber("utm_position", Odometry, self.curr_utm_callback)
        self.goal_UTM_subscriber = rospy.Subscriber("rover_state", StateMsg, self.goal_utm_callback)
        self.state_publisher = rospy.Publisher('/gps_checker_node/rover_state', StateMsg, queue_size=10)
        self.state_msg_old = StateMsg()
        self.updated_state_msg = StateMsg()
        self.curr_utm_pos = np.zeros((0,3))
        self.goal_utm_pos = np.zeros((0,3))

    
    def curr_utm_callback(self, curr_utm_msg):

        self.curr_utm_pos = np.asarray([curr_utm_msg.pose.position.x, curr_utm_msg.pose.position.y, curr_utm_msg.pose.position.z])

    def goal_utm_callback(self, goal_utm_msg):

        self.state_msg_old = goal_utm_msg

        self.goal_utm_pos = np.asarray([goal_utm_msg.curr_goal.pose.position.x, goal_utm_msg.curr_goal.pose.position.y, goal_utm_msg.curr_goal.pose.position.z])

    
    def check_goal_error(self):

        self.updated_state_msg = self.state_msg_old

        err = np.linalg.norm(self.curr_utm_pos - self.goal_utm_pos)

        if err < 2.5 :

            self.updated_state_msg.GPS_GOAL_REACHED = True

        self.state_publisher.publish(self.updated_state_msg)



    def main(self):
        rospy.init_node("gps_goal_checker")
        
        while not rospy.is_shutdown():

            self.check_goal_error()
        
        return 

if __name__ == "__main__":
    gps_goal_checker = GPSCheckerNode()    
    gps_goal_checker.main()