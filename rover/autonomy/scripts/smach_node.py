#!/usr/bin/env python3

import rospy
import smach 
import numpy as np
from smach_ros import SimpleActionState
from rsx_rover.msg import GPSMsg, StateMsg
from std_srvs.srv import Empty, EmptyResponse
import argparse
import yaml

class StateMachineNode():

    def __init__(self, args):

        self.state_subscriber = rospy.Subscriber(StateMsg, self.state_callback)
        self.MODE = "IDLE"
        self.GPS_GOALS = np.zeros(0,3)
        self.task_config = args.task_config
        self.GPS_counter = 0

        # State Machine States
        self.rover_idle = self.RoverIdle()
        self.rover_manual = self.RoverManual()
        self.rover_initialize = self.RoverInitialize()
        self.rover_start = self.RoverStart()
        self.rover_gps_traverse = self.RoverGPSTraverse()
        self.rover_aruco_traverse = self.RoverARucoTraverse()
        self.rover_scan = self.RoverScan()
        self.rover_gps_checkpoint = self.RoverGPSCheckpoint()
        self.rover_timeout = self.RoverTimeout()
        self.rover_mission_over = self.RoverMissionOver()

    def state_callback(self, state_msg):

        self.GPS_GOAL_REACHED = state_msg.GPS_GOAL_REACHED
        self.MODE = state_msg.rover_mode


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
        smach.State.__init__(self, outcomes=['MOVE_TO_IDLE','MOVE_TO_INTIALIZE', 'STAY'])
        self.counter = 0
        # Your state initialization goes here

    def execute(self, userdata):
        rospy.loginfo('Executing state MANUAL')
        # Your state execution goes here
        if self.MODE == 'MANUAL':
            return 'STAY'
        elif self.MODE == 'AUTONOMY':
            return 'MOVE_TO_INITIALIZE'
        elif self.MODE == 'IDLE':
            return 'MOVE_TO_IDLE'

class RoverInitialize(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        # Your state initialization goes here

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        # Your state execution goes here

        # Load the configuration file 

        with open(userdata.task_config, "r") as stream:
            try:
                config = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        if :
            return 'outcome1'
        else:
            return 'outcome2'

class RoverStart(smach.State):

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
    
class RoverGPSTraverse(smach.State):

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

class RoverARucoTraverse(smach.State):

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
        rospy.loginfo('Executing state Rovransition')
        

        if self.GPS_counter == self.num_GPS_goals - 1: 
            # 
            return "MOVE_TO_MISSION_OVER"

        if self.GPS_counter == 2 and not self.gps_location_read:
            # Move to node for determining GPS loc from heading and distance
            return 'DETERMINE_REL_GPS_GOAL'
        
        else:
            self.GPS_counter += 1
            # Load the next GPS goal into the state 
            # Move to RoverGPSTraverse
            return 'MOVE_TO_GPS_TRAVERSE'

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

class RoverTimeout(smach.State):

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

class RoverMissionOver(smach.State):

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

    state_machine_node = StateMachineNode(args)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['MISSION_OVER', 'MISSION_ABORTED'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('RoverIdle', state_machine_node.rover_idle(),
                               transitions={'STAY': 'RoverIdle',
                                            'MOVE_TO_INITIALIZE' : 'RoverInitialize'})
        smach.StateMachine.add('RoverManual', state_machine_node.rover_manual(),
                               transitions={'STAY': 'RoverManual',
                                            'MOVE_TO_INITIALIZE' : 'RoverInitialize', 
                                            'MOVE_TO_IDLE' : 'RoverIdle'})
        smach.StateMachine.add('RoverInitialize', RoverInitialize(), 
                               transitions={'LOADING' : 'RoverInitialize',
                                            'LOAD_COMPLETE' : 'RoverStart'})
        smach.StateMachine.add('RoverStart', state_machine_node.rover_start(), 
                               transitions={'GPS_GOAL_QUEUED':'RoverGPSTraverse'})
        smach.StateMachine.add('RoverGPSTraverse', state_machine_node.rover_gps_traverse(), 
                               transitions={'GPS_GOAL_REACHED':'RoverARucoTraverse'})
        smach.StateMachine.add('RoverGPSCheckpoint', state_machine_node.rover_gps_checkpoint(),
                               transitions={'ARUCO_FOUND':'RoverARucoTraverse', 
                                            'START_SEARCH':'RoverScan'})
        smach.StateMachine.add('RoverARucoTraverse', state_machine_node.rover_aruco_traverse(),
                               transitions={'GOAL_REACHED':'RoverTransition',
                                            'ARUCO_FOUND':'RoverARucoTraverse', 
                                            'SEARCHING': 'RoverScan'})
        smach.StateMachine.add('RoverScan', state_machine_node.rover_scan(),
                               transitions={'ARUCO_FOUND':'RoverARucoTraverse', 
                                            'SEARCHING': 'RoverScan'})
        smach.StateMachine.add('RoverTransition', state_machine_node.rover_transition(),
                               transitions={'ARUCO_FOUND':'RoverARucoTraverse', 
                                            'SEARCHING': 'RoverScan',
                                            'DETERMINE_REL_GPS_GOAL':'RoverReadGPSGoal',
                                            'MOVE_TO_GPS_TRAVERSE': 'RoverGPSTraverse'})
        smach.StateMachine.add('RoverReadGPSGoal', state_machine_node.rover_read_gps_goal(),
                               transitions={'LOC_CONFIRMED':'RoverTransition', 
                                            'SEARCHING': 'RoverScan',
                                            'DETERMINE_REL_GPS_GOAL':'RoverReadGPSGoal',
                                            'MOVE_TO_GPS_TRAVERSE': 'RoverGPSTraverse'})
        smach.StateMachine.add('RoverTimeout', state_machine_node.rover_timeout(),
                               transitions={'ARUCO_FOUND':'RoverARucoTraverse', 
                                            'SEARCHING': 'RoverScan'})
        smach.StateMachine.add('RoverMissionOver', state_machine_node.rover_mission_over(),
                               transitions={'ARUCO_FOUND':'RoverARucoTraverse', 
                                            'SEARCHING': 'RoverScan'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Import command line arguments for task')
    parser.add_argument('task_config', metavar='f', default="~/rover_ws/src/rsx-rover/task_config.json",
                        help='Task configuration file')

    args = parser.parse_args()
    main(args)