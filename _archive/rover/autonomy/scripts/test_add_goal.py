#!/usr/bin/env python3

import rospy
from rover.msg import StateMsg
from rover.srv import AddGoal 

if __name__ == "__main__":
    rospy.init_node("add_goal_test_node")
    rospy.wait_for_service('add_goal')
    client_request = rospy.ServiceProxy('add_goal', AddGoal)
    response = client_request(3, 0.5, 0, "gps", False)