#!/usr/bin/env python3

import rospy
import smach 
import numpy as np
from smach_ros import SimpleActionState
from rover.msg import StateMsg
from rover.srv import AddGoal, AddGoalResponse
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty, EmptyResponse
from tf.transformations import quaternion_from_euler
from add_goal import *
import argparse
import yaml
import json
import actionlib 
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class StateMachineNode:
    """
    Class for the RSX rover state machine
    """
    def __init__(self, num_gps_goals):

        self.state_subscriber = rospy.Subscriber('/rover_state', StateMsg, self.state_callback)
        self.state_publisher = rospy.Publisher('/sm_node/rover_state', StateMsg, queue_size=10)
        self.add_goal_srv = rospy.Service('add_goal', AddGoal, self.handle_add_goal)
        self.MODE = "IDLE"
        self.GPS_goals = np.zeros((1,3))
        self.curr_goal = PoseStamped()
        self.num_gps_goals = num_gps_goals
        self.GPS_counter = 0
        self.olat = 10
        self.olon = 50

        # State Machine States
        self.rover_idle = self.RoverIdle(self)
        self.rover_manual = self.RoverManual(self)
        self.rover_initialize = self.RoverInitialize(self)
        self.rover_transition = self.RoverTransition(self)
        self.rover_traverse = self.RoverTraverse(self)
        self.rover_aruco_traverse = self.RoverArucoTraverse(self)
        self.rover_gps_checkpoint = self.RoverGPSCheckpoint(self)
        self.rover_aruco_scan = self.RoverArucoScan(self)
        self.rover_light_beacon_scan = self.RoverLightBeaconScan(self)
        self.rover_transition = self.RoverTransition(self)

        self.rover_ground_search = self.RoverGroundSearch(self) # TEST CODE

    def state_callback(self, state_msg):
        """
        Callback for "/rover_state" topic 
        """
        self.state_msg_raw = state_msg
        self.GPS_GOAL_REACHED = state_msg.GPS_GOAL_REACHED
        self.MODE = state_msg.rover_mode
        self.GPS_GOALS = state_msg.GPS_goals
        self.ar_tag_detected = state_msg.AR_TAG_DETECTED
        self.curr_ar_tag = state_msg.curr_AR_ID
        self.MISSION_OVER = state_msg.MISSION_OVER
        self.aruco_scan_complete = state_msg.AR_SCANNED
    
    def handle_add_goal(self, req):
        """
        Handler for adding goals to the state machine
        """
        goal_type = req.type 
        give_priority = req.give_priority # If True, cue this as the next goal

        if goal_type == "gps":
            # Convert to a goal in metres in the odom frame
            get_gps = GPS_to_UTM(lat=req.x, lon=req.y)
            goal = get_gps.convertGPSToOdom()

            try:
                if goal == None:
                    return AddGoalResponse("Invalid coordinate format provided")
            except:
                rospy.loginfo("Goal successfully converted")
        
        elif goal_type == "light_beacon":
            # should be able to directly add to the goal queue
            # Add goal to queue
            self.state_msg_raw.LIGHT_BEACON_DETECTED = True
            goal = np.asarray[req.x, req.y, req.w]
            
            try:
                if goal == None:
                    return AddGoalResponse("Invalid coordinate format provided")
            except:
                rospy.loginfo("Goal successfully converted")
        
        elif goal_type == "radio_beacon":
            # Convert to a goal in metres in the odom frame
            self.state_msg_raw.RADIO_BEACON_DETECTED = True
            get_gps = GPS_to_UTM(req.x, req.y)
            goal = get_gps.convertGPSToOdom()
            
            try:
                if goal == None:
                    return AddGoalResponse("Invalid coordinate format provided")
            except:
                rospy.loginfo("Goal successfully converted")
        
        elif goal_type == "odom":
            # Add goal to queue
            try:
                goal = np.asarray([req.x, req.y, req.w])
            except:
                return "Invalid coordinate format provided"
        
        elif goal_type == "sign":
            
            self.state_msg_raw.SIGN_DETECTED = True
            goal = get_gps.convertSignToOdom(req.x, req.w)
            try:
                if goal == None:
                    return AddGoalResponse("Invalid coordinate format provided")
            except:
                rospy.loginfo("Goal successfully converted")
        
        else: 
            return "Invalid goal type provided"

        if give_priority: 

            self.GPS_goals = np.vstack((goal, self.GPS_goals))
        
        else:
            self.GPS_goals = np.vstack((self.GPS_goals, goal))

        return "Successfully added goal to the list"

    def convertGoals(self):

        goal_list = []

        rospy.loginfo(self.GPS_goals)

        for curr_goal in self.GPS_goals:

            goal = PoseStamped()
            goal.header.frame_id = "laser_odom" # change
            goal.header.stamp = rospy.Time.now()

            goal.pose.position.x = curr_goal[0]
            goal.pose.position.y = curr_goal[1]
            quat = quaternion_from_euler(0,0,curr_goal[2])
            goal.pose.orientation.x = quat[0]
            goal.pose.orientation.y = quat[1]
            goal.pose.orientation.z = quat[2]
            goal.pose.orientation.w = quat[3]

            goal_list.append(goal)
        
        return goal_list

    class RoverIdle(smach.State):
        """
        Initial state of the rover - must enter into a new mode
        """
        def __init__(self, sm_node):
            smach.State.__init__(self, outcomes=["MOVE_TO_MANUAL","MOVE_TO_INITIALIZE", "STAY"])
            self.sm = sm_node

        def execute(self, userdata):
            rospy.loginfo('Executing state IDLE')

            # Your state execution goes here
            if self.sm.MODE == "AUTONOMY":
                return "MOVE_TO_INITIALIZE"
            elif self.sm.MODE == "MANUAL":
                return "MOVE_TO_MANUAL"
            else:
                return "STAY"

    class RoverManual(smach.State):
        """
        Manual mode of the rover
        """
        def __init__(self, sm_node):
            smach.State.__init__(self, outcomes=['MOVE_TO_IDLE', 'MOVE_TO_INITIALIZE', 'STAY'])
            self.counter = 0
            self.sm = sm_node

        def execute(self, userdata):
            rospy.loginfo('Executing state MANUAL')
            if self.sm.MODE == 'MANUAL':
                rospy.loginfo("Enabling manual control mode")
                self.sm.state_msg_raw.MANUAL_ENABLED = True
                self.sm.state_publisher.publish(self.sm.state_msg_raw)
                return 'STAY'
            
            elif self.sm.MODE == 'AUTONOMY':
                rospy.loginfo("Initializing autonomy...")
                self.sm.state_msg_raw.MANUAL_ENABLED = False
                self.sm.state_publisher.publish(self.sm.state_msg_raw)
                return 'MOVE_TO_INITIALIZE'
            
            elif self.sm.MODE == 'IDLE':
                rospy.loginfo("Entering idle mode...")
                self.sm.state_msg_raw.MANUAL_ENABLED = False
                self.sm.state_publisher.publish(self.sm.state_msg_raw)
                return 'MOVE_TO_IDLE'

            else:
                rospy.loginfo("Enabling manual control mode")
                self.sm.state_msg_raw.MANUAL_ENABLED = True
                self.sm.state_publisher.publish(self.sm.state_msg_raw)
                return 'STAY'  

    class RoverInitialize(smach.State):
        """
        Initialization of autonomy mode
            1. Load any pre-determined gps goals 
            2. Check if everything is ready to move to traversal
        """
        def __init__(self, sm_node):
            smach.State.__init__(self, outcomes=['MOVE_TO_MANUAL', 'MOVE_TO_IDLE', 'MOVE_TO_TRAVERSE','STAY'])
            self.sm = sm_node

        def execute(self, userdata):
            rospy.loginfo('Executing state INITIALIZE')
            
            # Load any provided waypoints
            rospy.loginfo("Please load any provided GPS waypoints using the AddGoal service")
            result = input("When you are done inputting the waypoints, please type [Y]. Any other input will leave the rover in initialize mode.")
            if result == "Y":
                self.sm.state_msg_raw.GPS_goals = self.sm.convertGoals()
                self.sm.state_msg_raw.curr_goal = self.sm.state_msg_raw.GPS_goals.pop()
                self.sm.state_publisher.publish(self.sm.state_msg_raw)
                return "MOVE_TO_TRAVERSE"
            else:
                if self.sm.MODE == "MANUAL":
                    return 'MOVE_TO_MANUAL'
                elif self.sm.MODE == "IDLE":
                    return "MOVE_TO_IDLE"
                else:
                    return "STAY"

    class RoverTransition(smach.State):
        """
        The mode is entered after the Aruco tag has been detected 
        Transition to traversal mode or mission over
        """
        def __init__(self, sm_node):
            smach.State.__init__(self, outcomes=['MOVE_TO_MANUAL', 'MOVE_TO_IDLE', 'MOVE_TO_TRAVERSE', 'MISSION_OVER', "STAY"])
            self.counter = 0
            self.sm = sm_node

        def execute(self, userdata):
            rospy.loginfo('Executing state RoverTransition')
            self.sm.state_msg_raw.GPS_GOAL_REACHED = False
            self.sm.state_publisher.publish(self.sm.state_msg_raw)
            if self.sm.MODE == "MANUAL":
                return "MOVE_TO_MANUAL"
            
            if self.sm.MODE == "IDLE":
                return "MOVE_TO_IDLE"
            
            # Check if the expected number of GPS goals has been visited
            if self.sm.GPS_counter == self.sm.num_gps_goals - 1: 
                return "MISSION_OVER"

            print(f"The GPS goal {self.sm.curr_goal} \n was reached. Moving to next GPS goal.")
            print("Please use the AddGoal Service to add any goals that have been collected before moving forward.")
            check = input("Please type [Y] once you have completed this step.")

            if check == "Y":
                return "MOVE_TO_TRAVERSE"
            else:
                return "STAY"

            
    class RoverTraverse(smach.State):
        """
        When move base is in process of being executed
        """
        def __init__(self, sm_node):
            smach.State.__init__(self, outcomes=['MOVE_TO_MANUAL', 'MOVE_TO_IDLE', 'TRAVERSING', 'ARUCO_TRAVERSE', 'GPS_GOAL_REACHED'])
            self.counter = 0
            self.sm = sm_node

        def execute(self, userdata):            #THESE MUST BE REMOVED
            self.sm.GPS_GOAL_REACHED = True
            rospy.loginfo('Executing state Traverse')
            if self.sm.MODE == "MANUAL":
                return 'MOVE_TO_MANUAL'
            if self.sm.MODE == "IDLE":
                return 'MOVE_TO_IDLE'
            if self.sm.ar_tag_detected and not self.sm.GPS_GOAL_REACHED: #edit for gps
                return 'ARUCO_TRAVERSE'
            if self.sm.GPS_GOAL_REACHED:
                return 'GPS_GOAL_REACHED'
            else:
                return 'TRAVERSING'


    class RoverArucoTraverse(smach.State):
        """
        Only enter this state when the GPS goal has not been reached but the Aruco tag has been detected 
        """
        def __init__(self, sm_node):
            smach.State.__init__(self, outcomes=['GPS_GOAL_REACHED','ARUCO_TRAVERSING', 'MOVE_TO_MANUAL', 'MOVE_TO_IDLE'])
            self.counter = 0
            self.sm = sm_node


        def execute(self, userdata):
            rospy.loginfo('Executing state ArucoTraverse')
            #remove
            self.sm.GPS_GOAL_REACHED = True
            # Your state execution goes here
            if self.sm.GPS_GOAL_REACHED:
                return 'GPS_GOAL_REACHED'
            if self.sm.MODE == "MANUAL":
                return 'MOVE_TO_MANUAL'
            if self.sm.MODE == "IDLE":
                return 'MOVE_TO_IDLE'
            else:
                return 'ARUCO_TRAVERSING'

    class RoverGPSCheckpoint(smach.State):
        """
        When the current GPS goal has been reached
        """
        def __init__(self, sm_node):
            smach.State.__init__(self, outcomes=['MOVE_TO_MANUAL','MOVE_TO_IDLE','ARUCO_FOUND','SEARCHING'])
            self.counter = 0
            self.sm = sm_node

        def execute(self, userdata):
            rospy.loginfo('Executing state RoverGPSCheckpoint')

            if self.sm.MODE == "MANUAL":
                return "MOVE_TO_MANUAL"
            if self.sm.MODE == "IDLE":
                return "MOVE_TO_IDLE"
            
                        #remove these
            self.sm.ar_tag_detected = False
            # If the GPS goal has been reached and the AR tag has been detected, move to transition to start the next leg
            if self.sm.ar_tag_detected == True:

                return "ARUCO_FOUND"

            # If the AR has not been found, do an automatic routine to rotate the rover the try to locate the AR tag
            if self.sm.ar_tag_detected == False:
                
                return "SEARCHING"
             
    class RoverArucoScan(smach.State):

        def __init__(self, sm_node):
            smach.State.__init__(self, outcomes=['MOVE_TO_MANUAL','MOVE_TO_IDLE', 'SCAN_COMPLETE', 'STAY'])
            self.counter = 0
            self.sm = sm_node


        def execute(self, userdata):
            rospy.loginfo('Executing state ArucoScan')
            
            if self.sm.aruco_scan_complete:
                return 'SCAN_COMPLETE'  
            if self.sm.MODE == "MANUAL":
                return 'MOVE_TO_MANUAL'
            if self.sm.MODE == "IDLE":
                return 'MOVE_TO_IDLE'
            else:
                return 'STAY'      

    # Potentially removing this and moving to a Light Beacon scanning action  
    class RoverLightBeaconScan(smach.State):

        def __init__(self, sm_node):
            smach.State.__init__(self, outcomes=['SCAN_COMPLETE', 'STAY','MOVE_TO_MANUAL','MOVE_TO_IDLE'])
            self.counter = 0
            self.sm = sm_node


        def execute(self, userdata):
            rospy.loginfo('Executing stateLightBeaconScan')
            
            # Add the actionlib execution for Aruco scanning
            if self.sm.light_beacon_scan_complete:
                return 'SCAN_COMPLETE' 
            if self.sm.MODE == "MANUAL":
                return 'MOVE_TO_MANUAL'
            if self.sm.MODE == "IDLE":
                return 'MOVE_TO_IDLE'
            else:   
                return 'STAY' 


    # New States
    class RoverGroundSearch(smach.State):

        def __init__(self, sm_node):
            smach.State.__init__(self, outcomes=['GPS_GOAL_REACHED', 'MOVE_TO_MANUAL', 'MOVE_TO_IDLE', 'GROUND_SEARCH'])
            self.count = 0
            self.sm = sm_node

        def execute(self, userdata):
            rospy.loginfo('Executing state GroundSearch')

            # Your state execution goes here
            if self.sm.GPS_GOAL_REACHED: # CHANGE GPS TO OBJECT FOUND!!!
                return 'GPS_GOAL_REACHED'
            if self.sm.MODE == "MANUAL":
                return 'MOVE_TO_MANUAL'
            if self.sm.MODE == "IDLE":
                return 'MOVE_TO_IDLE'
            else:
                return 'GROUND_SEARCH'
            


def main():
    rospy.init_node('rsx_rover_state_machine')

    num_gps_goals = 4#rospy.get_param('~num_gps_goals')

    state_machine_node = StateMachineNode(num_gps_goals)
    state_machine_node.rover_idle.sm = state_machine_node
    state_machine_node.rover_manual.sm = state_machine_node
    state_machine_node.rover_initialize.sm = state_machine_node
    state_machine_node.rover_transition.sm = state_machine_node
    state_machine_node.rover_traverse.sm = state_machine_node
    state_machine_node.rover_aruco_traverse.sm = state_machine_node
    state_machine_node.rover_gps_checkpoint.sm = state_machine_node
    state_machine_node.rover_aruco_scan.sm = state_machine_node
    state_machine_node.rover_light_beacon_scan.sm = state_machine_node
    state_machine_node.rover_transition.sm = state_machine_node   

    state_machine_node.rover_ground_search.sm = state_machine_node  

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['MISSION_OVER'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('RoverIdle', state_machine_node.rover_idle,
                               transitions={'STAY': 'RoverIdle',
                                            'MOVE_TO_INITIALIZE' : 'RoverInitialize',
                                            'MOVE_TO_MANUAL': 'RoverManual'})
        smach.StateMachine.add('RoverManual', state_machine_node.rover_manual,
                               transitions={'STAY': 'RoverManual',
                                            'MOVE_TO_INITIALIZE' : 'RoverInitialize', 
                                            'MOVE_TO_IDLE' : 'RoverIdle'})
        smach.StateMachine.add('RoverInitialize', state_machine_node.rover_initialize, 
                               transitions={'STAY' : 'RoverInitialize',
                                            'MOVE_TO_TRAVERSE' : 'RoverTraverse',
                                            'MOVE_TO_MANUAL': 'RoverManual',
                                            'MOVE_TO_IDLE': 'RoverIdle'})
        smach.StateMachine.add('RoverTransition', state_machine_node.rover_transition,
                               transitions={'MOVE_TO_TRAVERSE':'RoverTraverse', 
                                            'STAY': 'RoverTransition',
                                            'MOVE_TO_MANUAL': 'RoverManual',
                                            'MOVE_TO_IDLE': 'RoverIdle'})
        smach.StateMachine.add('RoverTraverse', state_machine_node.rover_traverse, 
                               transitions={'GPS_GOAL_REACHED':'RoverGPSCheckpoint',
                                            'ARUCO_TRAVERSE':'RoverArucoTraverse',
                                            'TRAVERSING':'RoverTraverse',
                                            'MOVE_TO_MANUAL': 'RoverManual',
                                            'MOVE_TO_IDLE': 'RoverIdle'})
        smach.StateMachine.add('RoverArucoTraverse', state_machine_node.rover_aruco_traverse, 
                               transitions={'GPS_GOAL_REACHED':'RoverGPSCheckpoint',
                                            'ARUCO_TRAVERSING':'RoverArucoTraverse',
                                            'MOVE_TO_MANUAL': 'RoverManual',
                                            'MOVE_TO_IDLE': 'RoverIdle'})
        smach.StateMachine.add('RoverGPSCheckpoint', state_machine_node.rover_gps_checkpoint,
                               transitions={'ARUCO_FOUND':'RoverTransition',
                                            'SEARCHING':'RoverArucoScan',
                                            'MOVE_TO_MANUAL': 'RoverManual',
                                            'MOVE_TO_IDLE': 'RoverIdle'})
        smach.StateMachine.add('RoverArucoScan', state_machine_node.rover_aruco_scan,
                               transitions={'SCAN_COMPLETE':'RoverTransition', 
                                            'MOVE_TO_MANUAL': 'RoverManual',
                                            'MOVE_TO_IDLE': 'RoverIdle', 
                                            'STAY':'RoverArucoScan'})
        smach.StateMachine.add('RoverLightBeaconScan', state_machine_node.rover_light_beacon_scan,
                               transitions={'SCAN_COMPLETE':'RoverTransition', 
                                            'MOVE_TO_MANUAL': 'RoverManual',
                                            'MOVE_TO_IDLE': 'RoverIdle', 
                                            'STAY':'RoverLightBeaconScan'})
        # TEST CODE
        smach.StateMachine.add('RoverGroundSeach', state_machine_node.rover_ground_search, 
                                transitions={'GPS_GOAL_REACHED':'RoverGroundSeach', 
                                            'MOVE_TO_MANUAL': 'RoverGroundSeach',
                                            'MOVE_TO_IDLE': 'RoverGroundSeach', 
                                            'GROUND_SEARCH':'RoverGroundSeach'})


    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':

    main()