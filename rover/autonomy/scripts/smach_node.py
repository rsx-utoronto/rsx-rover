#!/usr/bin/env python3

import rospy
import smach 
import numpy as np
from smach_ros import SimpleActionState
from rover.msg import StateMsg
from rover.srv import AddGoal
from std_srvs.srv import Empty, EmptyResponse
from add_goal import *
import argparse
import yaml
import json

class StateMachineNode():

    def __init__(self, num_gps_goals):

        self.state_subscriber = rospy.Subscriber(StateMsg, self.state_callback)
        self.state_publisher = rospy.Publisher('/rover_state', StateMsg, queue_size=10)
        self.add_goal_srv = rospy.Service('add_goal', AddGoal, self.handle_add_goal)
        self.MODE = "IDLE"
        self.GPS_goals = np.zeros(0,3)
        self.num_gps_goals = num_gps_goals
        self.GPS_counter = 0

        # State Machine States
        self.rover_idle = self.RoverIdle()
        self.rover_manual = self.RoverManual()
        self.rover_initialize = self.RoverInitialize()
        self.rover_traverse = self.RoverTraverse()
        self.rover_scan = self.RoverScan()
        self.rover_transition = self.RoverTransition()

    def state_callback(self, state_msg):
        self.state_msg_raw = state_msg
        self.GPS_GOAL_REACHED = state_msg.GPS_GOAL_REACHED
        self.MODE = state_msg.rover_mode
        self.GPS_GOALS = state_msg.GPS_goals
        self.ar_tag_detected = state_msg.AR_TAG_DETECTED
        self.curr_ar_tag = self.curr_AR_ID
        self.MISSION_OVER = self.MISSION_OVER
    
    def handle_add_goal(self, req):

        goal_type = req.type 
        goal_coords = json.loads(req.coordinates)
        give_priority = req.give_priority # If True, cue this as the next goal

        response_status = False

        if goal_type is "gps":
            # Convert to a goal in metres in the odom frame
            goal = convertGPSToOdom(goal_coords)
        
        elif goal_type is "light_beacon":
            # should be able to directly add to the goal queue
            # Add goal to queue
            self.state_msg_raw.LIGHT_BEACON_DETECTED = True
            goal = np.asarray[goal_coords.x, goal_coords.y, goal_coords.w]
            if goal is False: 
                return "Invalid coordinate format provided"
        
        elif goal_type is "radio_beacon":
            # Convert to a goal in metres in the odom frame
            self.state_msg_raw.RADIO_BEACON_DETECTED = True
            goal = convertGPSToOdom(goal_coords)
            if goal is False: 
                return "Invalid coordinate format provided"
        
        elif goal_type is "odom":

            # Add goal to queue
            try:
                goal = np.asarray[goal_coords.x, goal_coords.y, goal_coords.w]
            except:
                return "Invalid coordinate format provided"
        
        elif goal_type is "sign":
            
            self.state_msg_raw.SIGN_DETECTED = True
            goal = convertSignToOdom(goal_coords)
            if goal is False: 
                return "Invalid coordinate format provided"
        
        else: 
            return "Invalid goal type provided"

        if give_priority: 

            self.GPS_goals = np.vstack((goal, self.GPS_goals))
        
        else:
            self.GPS_goals = np.vstack((self.GPS_goals, goal))
        
        return "Successfully added goal to list"


class RoverIdle(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=["MOVE_TO_MANUAL","MOVE_TO_INTIALIZE", "STAY"])

    def execute(self, userdata):
        rospy.loginfo('Executing state IDLE')

        # Your state execution goes here
        if self.MODE == "IDLE":
            return "STAY"
        elif self.MODE == "AUTONOMY":
            return "MOVE_TO_INTIALIZE"
        elif self.MODE == "MANUAL":
            return "MOVE_TO_MANUAL"

class RoverManual(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['MOVE_TO_IDLE', 'MOVE_TO_INTIALIZE', 'STAY'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state MANUAL')
        if self.MODE == 'MANUAL':
            rospy.info("Enabling manual control mode")
            self.state_msg_raw.MANUAL_ENABLED = True
            return 'STAY'
        
        elif self.MODE == 'AUTONOMY':
            rospy.info("Initializing autonomy...")
            self.state_msg_raw.MANUAL_ENABLED = False
            return 'MOVE_TO_INITIALIZE'
        
        elif self.MODE == 'IDLE':
            rospy.info("Entering idle mode...")
            self.state_msg_raw.MANUAL_ENABLED = False
            return 'MOVE_TO_IDLE'

class RoverInitialize(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['MOVE_TO_MANUAL', 'MOVE_TO_TRANSITION','STAY'])

    def execute(self, userdata):
        rospy.loginfo('Executing state INITIALIZE')
        
        # Load any provided waypoints
        rospy.info("Please load any provided GPS waypoints using the AddGoal service")
        result = input("When you are done inputting the waypoints, please type [Y]. Any other input will leave the rover in initialize mode.")
        if result is "Y":
            return "MOVE_TO_TRANSITION"
        else:
            if self.MODE == "MANUAL":
                return 'MOVE_TO_MANUAL'
            else:
                return "STAY"

class RoverTransition(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['ARUCO_FOUND','SEARCHING'])
        self.counter = 0
        # Your state initialization goes here

    def execute(self, userdata):
        rospy.loginfo('Executing state RoverTransition')
        
        # Check if the expected number of GPS goals has been visited
        if userdata.GPS_counter == userdata.num_GPS_goals - 1: 
            return "MISSION_OVER"

        if userdata.GPS_counter == 2 and not userdata.gps_location_read:
            # Move to node for determining GPS loc from heading and distance
            return 'DETERMINE_REL_GPS_GOAL'
        
        else:
            userdata.GPS_counter += 1
            # Load the next GPS goal into the state 
            # Move to RoverTraverse
            return 'MOVE_TO_TRAVERSE'
        
class RoverTraverse(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['MOVE_TO_MANUAL','TRAVERSING','GPS_GOAL_REACHED'])
        self.counter = 0
        # Your state initialization goes here

    def execute(self, userdata):
        rospy.loginfo('Executing state Traverse')
        
        if self.MODE == "MANUAL":
            return 'MOVE_TO_MANUAL'
        if self.GPS_GOAL_REACHED:
            return 'GPS_GOAL_REACHED'
        else:
            return 'TRAVERSING'

# Potentially removing this and moving to an action client for aruco detection node
class RoverARucoScan(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        # Your state initialization goes here

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        # Your state execution goes here
        if xxxx:
            return 'outcome1'
        else:
            return 'outcome2'

class RoverGPSCheckpoint(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        # Your state initialization goes here

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        # Your state execution goes here
        if xxxx:
            return 'outcome1'
        else:
            return 'outcome2'
    
class RoverTransition(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        # Your state initialization goes here

    def execute(self, userdata):
        rospy.loginfo('Executing state RoverTransition')
        
        # Check if the expected number of GPS goals has been visited
        if userdata.GPS_counter == userdata.num_GPS_goals - 1: 
            return "MISSION_OVER"

        if userdata.GPS_counter == 2 and not userdata.gps_location_read:
            # Move to node for determining GPS loc from heading and distance
            return 'DETERMINE_REL_GPS_GOAL'
        
        else:
            userdata.GPS_counter += 1
            # Load the next GPS goal into the state 
            # Move to RoverTraverse
            return 'MOVE_TO_TRAVERSE'
        

# Potentially removing this and moving to a Light Beacon scanning action  
class RoverScan(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        # Your state initialization goes here

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        # Your state execution goes here
        if xxxx:
            return 'outcome1'
        else:
            return 'outcome2'


def main(args):
    rospy.init_node('rsx_rover_state_machine')

    state_machine_node = StateMachineNode(args.num_gps_goals)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['MISSION_OVER', 'MISSION_ABORTED'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('RoverIdle', state_machine_node.rover_idle(),
                               transitions={'STAY': 'RoverIdle',
                                            'MOVE_TO_INITIALIZE' : 'RoverInitialize', 
                                            'MOVE_TO_MANUAL': 'RoverManual'})
        smach.StateMachine.add('RoverManual', state_machine_node.rover_manual(),
                               transitions={'STAY': 'RoverManual',
                                            'MOVE_TO_INITIALIZE' : 'RoverInitialize', 
                                            'MOVE_TO_IDLE' : 'RoverIdle'})
        smach.StateMachine.add('RoverInitialize', RoverInitialize(), 
                               transitions={'STAY' : 'RoverInitialize',
                                            'MOVE_TO_TRANSITION' : 'RoverTransition',
                                            'MOVE_TO_MANUAL': 'RoverManual'})
        smach.StateMachine.add('RoverTraverse', state_machine_node.rover_gps_traverse(), 
                               transitions={'GPS_GOAL_REACHED':'RoverGPSCheckpoint'})
        smach.StateMachine.add('RoverGPSCheckpoint', state_machine_node.rover_gps_checkpoint(),
                               transitions={'ARUCO_FOUND':'RoverTransition', 
                                            'START_SEARCH':'RoverScan'})
        smach.StateMachine.add('RoverScan', state_machine_node.rover_scan(),
                               transitions={'ARUCO_FOUND':'RoverTraverse', 
                                            'SEARCHING': 'RoverScan'})
        smach.StateMachine.add('RoverTransition', state_machine_node.rover_transition(),
                               transitions={'ARUCO_FOUND':'RoverTraverse', 
                                            'SEARCHING': 'RoverScan'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Import command line arguments for task')
    # parser.add_argument('task_config', metavar='f', default="~/rover_ws/src/rsx-rover/task_config.json",
    #                      help='Task configuration file')
    parser.add_argument('num_gps_goals', metavar='n', default=7,
                         help='Input the number of GPS goals expected (in total)')
    args = parser.parse_args()
    main(args)