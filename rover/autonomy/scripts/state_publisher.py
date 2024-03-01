#!/usr/bin/env python3

import rospy
from rover.msg import StateMsg
from rover.srv import AddGoal
from std_msgs.msg import String

class StateMachineNode:

    def __init__(self):

        self.state_msg = StateMsg()
        self.state_msg.rover_mode = "AUTONOMY"#"MANUAL"

        self.state_pub = rospy.Publisher("/rover_state", StateMsg, queue_size=1000)
        self.gps_state_sub = rospy.Subscriber("/gps_checker_node/rover_state", StateMsg, self.gps_state_callback)
        self.aruco_state_sub = rospy.Subscriber("/aruco_node/rover_state", StateMsg, self.aruco_state_callback)
        self.aruco_scanned_sub = rospy.Subscriber("aruco_scanned_node/rover_state", StateMsg, self.aruco_scanned_callback)
        self.light_state_sub = rospy.Subscriber("/light_node/rover_state", StateMsg, self.aruco_state_callback)
        self.radio_state_sub = rospy.Subscriber("/radio_node/rover_state", StateMsg, self.radio_state_callback)
        self.gui_state_sub = rospy.Subscriber("/gui_node/rover_state", StateMsg, self.gui_state_callback)
        self.sm_state_sub = rospy.Subscriber("/sm_node/rover_state", StateMsg, self.sm_state_callback)
        self.ground_state_sub = rospy.Subscriber("/ground_node/rover_state", StateMsg, self.ground_state_callback)

    def gps_state_callback(self, msg):
        self.state_msg.GPS_GOAL_REACHED = msg.GPS_GOAL_REACHED

    def aruco_state_callback(self, msg):
        self.state_msg.curr_AR_ID = msg.curr_AR_ID
        self.state_msg.AR_TAG_DETECTED = msg.AR_TAG_DETECTED

    def aruco_scanned_callback(self, msg):
        self.state_msg.AR_SCANNED = msg.AR_SCANNED

    def radio_state_callback(self, msg):
        self.state_msg.RADIO_BEACON_DETECTED_BEACON_DETECTED = msg.RADIO_BEACON_DETECTED
        self.state_msg.radio_beacon_goal = msg.radio_beacon_goal

    def ground_state_callback(self, msg):
        self.state_msg.GROUND_OBJECT_FOUND = msg.GROUND_OBJECT_FOUND
        self.state_msg.GROUND_OBJECT_REACHED = msg.GROUND_OBJECT_REACHED

    def gui_state_callback(self, msg):
        self.state_msg.rover_mode = msg.rover_mode
    
    def light_state_callback(self, msg):
        self.state_msg.LIGHT_BEACON_DETECTED = msg.LIGHT_BEACON_DETECTED 
        self.state_msg.light_beacon_goal = msg.light_beacon_goal
    
    def light_state_callback(self, msg):
        self.state_msg.LIGHT_BEACON_DETECTED = msg.LIGHT_BEACON_DETECTED 
        self.state_msg.light_beacon_goal = msg.light_beacon_goal

    def sm_state_callback(self, msg):
        self.state_msg.MANUAL_ENABLED = msg.MANUAL_ENABLED
        self.state_msg.GPS_goals = msg.GPS_goals
        self.state_msg.curr_goal = msg.curr_goal
    
    
def main():
    rospy.init_node("state_publisher_node")

    state_node = StateMachineNode()

    while not rospy.is_shutdown():

        state_node.state_pub.publish(state_node.state_msg)
    
    rospy.spin()


if __name__ == "__main__":
    main()



