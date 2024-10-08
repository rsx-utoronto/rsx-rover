#!/usr/bin/env python3

import rospy
import smach 
import time
from smach_ros import SimpleActionState
from rover.msg import StateMsg
from nav_msgs.msg import Odometry
import numpy as np


class GPSCheckerNode():

    def __init__(self):
        # CHANGE
        self.state_publisher = rospy.Publisher('/gps_checker_node/rover_state', StateMsg, queue_size=10)
        t = time.time()
        while (time.time() - t) < 15:
            print("Passing time")
            pass
        self.curr_subscriber = rospy.Subscriber("/stereo_odometry", Odometry, self.curr_callback)
        self.goal_subscriber = rospy.Subscriber("/rover_state", StateMsg, self.goal_callback)
        self.state_msg_old = StateMsg()
        self.updated_state_msg = StateMsg()
        self.curr_pos = np.zeros((0,3))
        self.goal_pos = np.zeros((0,3))

    
    def curr_callback(self, curr_msg):

        self.curr_pos = np.asarray([curr_msg.pose.position.x, curr_msg.pose.position.y, curr_msg.pose.position.z])

    def goal_callback(self, goal_msg):

        self.state_msg_old = goal_msg

        self.goal_pos = np.asarray([goal_msg.curr_goal.pose.position.x, goal_msg.curr_goal.pose.position.y, goal_msg.curr_goal.pose.position.z])

    
    def check_goal_error(self):

        self.updated_state_msg = self.state_msg_old

        err = np.linalg.norm(self.curr_pos - self.goal_pos)

        if err < 2.5 and len(self.updated_state_msg.GPS_goals) > 0:

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