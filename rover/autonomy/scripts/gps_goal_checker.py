#!/usr/bin/env python3

import rospy
import smach 
from smach_ros import SimpleActionState
from rsx_rover.msg import GPSMsg, StateMsg
from nav_msgs.msg import Odometry
import numpy as np


class GPSCheckerNode():

    def __init__(self):

        self.curr_UTM_subscriber = rospy.Subscriber("utm_position", Odometry, self.curr_utm_callback)
        self.goal_UTM_subscriber = rospy.Subscriber(StateMsg, self.goal_utm_callback)
        self.state_publisher = rospy.Publisher('rover_state', StateMsg, queue_size=10)

    
    def curr_utm_callback(self, curr_utm_msg):

        self.curr_utm_pos = curr_utm_msg.pose.position

    def goal_utm_callback(self, goal_utm_msg):

        self.state_msg_old = goal_utm_msg

        self.goal_utm_pos = goal_utm_msg.utm_frame.position

    
    def check_goal_error(self):

        self.updated_state_msg = StateMsg()
        self.updated_state_msg = self.state_msg_old

        err = np.linalg.norm(self.curr_utm_pos - self.goal_utm_pos)

        if err < 2.5 :

            self.updated_state_msg.GPS_GOAL_REACHED = True

        self.state_publisher.pub(self.updated_state_msg)



    def main(self):

        while not rospy.is_shutdown():

            self.check_goal_error()
        
        return 

if __name__ == "__main__":
    gps_goal_checker = GPSCheckerNode()    
    gps_goal_checker.main()