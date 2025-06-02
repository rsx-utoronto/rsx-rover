#!/usr/bin/python3
"""
Code for the state machine

"""
import rospy
import object_subscriber_node
# import led_light
import smach
import time
import math
from optimal_path import OPmain
from thomas_grid_search import thomasgrid
import ar_detection_node  
from std_msgs.msg import Float32MultiArray, Bool, Float64MultiArray
from geometry_msgs.msg import Twist
from sm_straight_line import StraightLineApproach
from sm_straight_line_new import StraightLineApproachNew
from astar_obstacle_avoidance_algorithim import AstarObstacleAvoidance
import astar_obstacle_avoidance_grid_search 
import gps_conversion_functions as functions
import gps_to_pose as gps_to_pose
import sm_grid_search
import ar_detection_node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import yaml
import os

file_path = os.path.join(os.path.dirname(__file__), "sm_config.yaml")

with open(file_path, "r") as f:
    sm_config = yaml.safe_load(f)


RUN_STATES_DEFAULT = ["GNSS1", "AR1", "OBJ2", "OBJ1", "AR2", "AR3", "GNSS2"] 
RUN_STATES = sm_config.get("RUN_STATES", RUN_STATES_DEFAULT)
print("run states from yaml", RUN_STATES)

#When it completes a task it publishes the message "Goal point reached: <Task Name>" to gui_status

#Example locations
#locations = {
#    "start": (43.6532, 79.3832), # Toronto
#    "GNSS1": (40.6892, -74.0445),   # Statue of Liberty
#    "GNSS2": (48.8584, 2.2945),     # Eiffel Tower
#    "AR1": (-33.8568, 151.2153),    # Sydney Opera House
#    "AR2": (40.4319, 116.5704),     # Great Wall of China (Mutianyu)
#    "AR3": (-22.9519, -43.2105),    # Christ the Redeemer
#    "OBJ1": (27.1751, 78.0421),     # Taj Mahal
#    "OBJ2": (41.8902, 12.4922),     # Colosseum
#}


# ROVER USES STRAIGHT LINE TRAVERSAL TO GET TO ARUCO POINT
# once it is AT the location for AR1/2, the two files need to run IMMEDIATELY and SIMULTANEOUSLY: 
# 1) ar_detection node (constantly looking for aruco), sm_grid_search (moving rover around in a square spiral) 
# 1a) aruco is detected by ar_detection node. --> grid search is stopped, IMMEDIATELY (add stop feature) + aruco_homing.
# 1b) aruco is Never found by ar_detection_node, meaning grid search will run until it's done. (red led - FAILED)
# 2) at any point that this task is done, regardless of pass/fail, make this a function.

# ROVER USES STRAIGHT LINE TRAVERSAL TO GET TO OBJECT (MALLET)
# once it is AT the location for AR1/2, the two files need to run IMMEDIATELY and SIMULTANEOUSLY: 
# 1) object_subscriber node (investigate), sm_grid_search (moving rover around in a square spiral) 
# 1a) object is detected by object_sub node. --> grid search is stopped, IMMEDIATELY (add stop feature) + aruco_homing.
# 1b) object is Never found by object_sub_node, meaning grid search will run until it's done. (red led - FAILED)

# HIGH LEVEL - using classes for all of this - look at the main functions for how to run/call
# You SUB to detection nodes (AR, object sub) to figure out if object is found
# You use GRID SEARCH to move around to try and be close enough to the object for the detection to trigger
# You use homing to get close enough to the object to count for points (ie. 1 m)
# FOR HOMING SPECIFICALLY - need Aimer, and AimerROS () - BBOX (subscribe to different box)



def shortest_path(start: str, locations: dict) -> list:
    return OPmain(start, locations)

#*******************************************************

# GLOB_MSGS class which includes all publishers, subscribers, callback functions, and the instances of AR and object detection nodes
# which will be used throughout the code.
# Publishers: gui_status(String) [For publishing status messages to the GUI], state(String) [For indicating what state we are in], led_light(String)
# [For publishing messages for led to light on]
# Subscribers : pose(PoseStamped) [Gets the pose/location data], /long_lat_goal_array(Float32MultiArray) [Gets the GPS coordinates of task points]

class GLOB_MSGS:
    def __init__(self):
        self.pub = rospy.Publisher("gui_status", String, queue_size=10)
        self.state_publisher = rospy.Publisher("state", String, queue_size=10) #ask about que size
        self.led_publisher = rospy.Publisher("led_light", String, queue_size = 10)
        self.sub = rospy.Subscriber("pose", PoseStamped, self.pose_callback)
        self.odom_sub = rospy.Subscriber("/rtabmap/odom", Odometry, self.odom_callback) #Subscribes to the pose topic
        self.gui_loc = rospy.Subscriber('/long_lat_goal_array', Float32MultiArray, self.coord_callback) 
        self.abort_sub = rospy.Subscriber("auto_abort_check", Bool, self.abort_callback)
        self.next_state_sub = rospy.Subscriber("/next_state", Bool, self.next_task_callback)
        self.next_task_check = False
        self.abort_check = False
        self.locations = None
        self.cartesian = None
        self.odom_zero = None
        self.done_early = False
        self.done_early=rospy.Subscriber("done_early", Bool, self.done_early_callback)
        self.ar_detection_node = ar_detection_node.ARucoTagDetectionNode() #Initializes the AR detection node
        self.object_detector_node = object_subscriber_node.ObjectDetectionNode() #Initializes the Object detection node
        # self.led_light = led_light.LedLight() #Initializes class for led light
        
    def pose_callback(self, msg):
        self.pose = msg
        # print(self.pose.header.stamp.secs, "in pose_callback")

    def abort_callback(self,msg):
        self.abort_check = msg.data
    
    def next_task_callback(self, msg):
        self.next_task_check = msg.data
    
    def get_abort_check(self):
        return self.abort_check
    
    def done_early_callback(self, msg):
        self.done_early = msg.data
    
    def get_next_task_check(self):
        return self.next_task_check
        
    def get_pose(self):
        return self.pose
    
    def odom_callback(self, data): #Callback function for the odometry subscriber
        self.odom = data
        if self.odom_zero is None:
            self.odom_zero = data.pose.pose.position
        
        relative_x = data.pose.pose.position.x - self.odom_zero.x
        relative_y = data.pose.pose.position.y - self.odom_zero.y
        relative_z = data.pose.pose.position.z - self.odom_zero.z

        self.current_position = (relative_x, relative_y, relative_z)
            
    
    def get_odom(self):
        return self.current_position

    def coord_callback(self, data): 
        location_data = data 
        if (len(location_data.data) == 16): #Process all 8 GPS coordinates

            locations = {} #Create empty locations dict
            location_name_list = ["start", "GNSS1", "GNSS2", "AR1", "AR2", "AR3", "OBJ1", "OBJ2"]
            i = 0
            for name in location_name_list:
                if location_data.data[i] is not None and location_data.data[i+1] is not None:
                    locations[name] = (location_data.data[i], location_data.data[i+1]) # For each mission name key, assign the (x,y) value tuple as element
                    i +=2
            
            self.locations = locations #assign the GPS coordinate dict to locations
        
        self.pub_state("Received GPS coordinates")

    def pub_state(self, state): #for publishing a message through the state publisher
        self.pub.publish(state)

    def pub_state_name(self, state): #for publishing state names for detecting aruco/objects 
        self.state_publisher.publish(state)
    
    def pub_led_light(self, msg): #for publishing signals for the leds
        self.led_publisher.publish(msg)
    

class InitializeAutonomousNavigation(smach.State): #State for initialization
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes = ["Tasks Execute"],
                             input_keys = ["cartesian", "start_location"],
                             output_keys = ["cartesian", "start_location"])
        self.glob_msg = None
        
    def set_msg(self, glob_msg: GLOB_MSGS):
        self.glob_msg = glob_msg
    

    def initialize(self, userdata): # main init function

        rospy.Subscriber("/long_lat_goal_array", Float32MultiArray, self.glob_msg.coord_callback) #Subcribes to the gui location publisher
        while (self.glob_msg.locations is None and rospy.is_shutdown() == False): #Waits for all GPS locations to be received
            time.sleep(10)
            self.glob_msg.pub_state("Waiting for GPS coordinates")

        cartesian_path = self.glob_msg.locations
        # cartesian_path = shortest_path('start', self.glob_msg.locations) #Generates the optimal path 
        # print(cartesian_path)
        
        self.glob_msg.pub_state("Changing GPS coordinates to cartesian") 
        cartesian = {}
        for el in cartesian_path: #For each location stated in the yaml file, it transforms the GPS coordinates to Cartesian coordinates
            if (el in RUN_STATES) or (el == "start"):
                distance = functions.getDistanceBetweenGPSCoordinates((self.glob_msg.locations["start"][0], self.glob_msg.locations["start"][1]), (self.glob_msg.locations[el][0], self.glob_msg.locations[el][1]))
                theta = functions.getHeadingBetweenGPSCoordinates(self.glob_msg.locations["start"][0], self.glob_msg.locations["start"][1], self.glob_msg.locations[el][0], self.glob_msg.locations[el][1])
                # since we measure from north y is r*cos(theta) and x is -r*sin(theta)
                x = distance * math.sin(theta)
                y = distance * math.cos(theta)

                cartesian[el] = (x,y) 
                self.glob_msg.pub_state(str(cartesian[el]))
        
        print("Before CARTESIAN", cartesian)
        
        cartesian_dict = {}
        
        if sm_config.get("run_in_order"): #If indicated, we assign the path same as the one in the yaml file
            cartesian_list = sm_config.get("States_run_in_order") 
        else: 
            cartesian_list = shortest_path('start', cartesian) #Else create optimal path
    
        for cart in cartesian_list:
            cartesian_dict[cart] = cartesian[cart]
            
        print("At CARTESIAN", cartesian_dict)
        
        self.glob_msg.cartesian = cartesian_dict #assigns the cartesian dict to cartesian to make it a global variable 
        
        gps_to_pose.GPSToPose(self.glob_msg.locations['start'], tuple(sm_config.get("origin_pose", [0.0,0.0])), tuple(sm_config.get("heading_vector", [1.0, 0.0]))) #creates an instance of GPSToPose to start publishing pose
    
    def execute(self, userdata): 
        self.glob_msg.pub_led_light("auto") #Initializes red led light
        self.glob_msg.pub_state("Initializing Autonomous Navigation")
        self.initialize(userdata)
        return "Tasks Execute"

        
class LocationSelection(smach.State): #State for determining which mission/state to go to and traversing there
    def __init__(self):
        smach.State.__init__(self, outcomes = RUN_STATES + ["Tasks Ended", "ABORT"],
                             input_keys = ["rem_loc_dict", "locations_dict", 'prev_loc', 'start_location'],
                             output_keys = ["prev_loc", "locations_dict", "rem_loc_dict", "start_location", "aborted_state"])
        self.glob_msg = None
        
    def set_msg(self, glob_msg: GLOB_MSGS):
        self.glob_msg = glob_msg
  
    def execute(self, userdata):
        print("start execture loc select")
        self.glob_msg.pub_state("Performing Next Task")
        if userdata.prev_loc == "start": #if its the first instance of running the state, initialize the global variables locations_dict, rem_loc_dict
            print("user_data is none")
            userdata.locations_dict = self.glob_msg.cartesian.copy() #loc_name, (lat, lon)
            print ("THIS IS THE CARTESIAN DICT", userdata.locations_dict)
            userdata.rem_loc_dict = userdata.locations_dict.copy()
            print (userdata.rem_loc_dict)
            userdata.start_location = (0, 0)
            
        path = userdata.rem_loc_dict
        if path != {}: #Checks if all locations are visited
            
            try:
                self.glob_msg.pub_state(f"Navigating to {list(path.items())[0][0]}") 
                self.glob_msg.pub_state(f"Navigating to {self.glob_msg.cartesian[list(path.items())[0][0]]}") 
                target = path[list(path.items())[0][0]]
                target_name = list(path.items())[0][0]
                self.glob_msg.pub_state_name(target_name) 
                print("target_name", target_name) # add check for if it's going to AR3, OBJ2 or leaving from those!
                if target_name == "AR3" or target_name == 'OBJ2' or userdata.prev_loc =='AR3' or userdata.prev_loc =='OBJ2':
                    print("doing obstacle_avoidance in straight line")
                    sla = AstarObstacleAvoidance(sm_config.get("straight_line_obstacle_lin_vel"), sm_config.get("straight_line_obstacle_ang_vel"), [target])
                else:
                    print("Not Doing Obstalce Avoidance")
                    #sla = StraightLineObstacleAvoidance(sm_config.get("straight_line_obstacle_lin_vel"), sm_config.get("straight_line_obstacle_ang_vel"), [target])
                   # sla = StraightLineApproach(sm_config.get("straight_line_approach_lin_vel"), sm_config.get("straight_line_approach_ang_vel"), [target]) 
                    if target_name=="GNSS1" or target_name=="GNSS2" or target_name=="start":
                        sla = StraightLineApproach(sm_config.get("straight_line_approach_lin_vel"), sm_config.get("straight_line_approach_ang_vel"), [target]) 
                    else:
                        sla = StraightLineApproachNew(sm_config.get("straight_line_approach_lin_vel"), sm_config.get("straight_line_approach_ang_vel"), [target], target_name) 
                sla.navigate() #navigating to the next mission on our optimal path, can have abort be called in the SLA file
                if self.glob_msg.abort_check: #Checks if abort button is pressed
                    userdata.aborted_state = list(path.items())[0][0]
                    return "ABORT"
            except rospy.ROSInterruptException:
                self.glob_msg.pub_state(f"ROS Interrupt Exception during Location Selection")
                if self.glob_msg.abort_check:
                    userdata.aborted_state = list(path.items())[0][0]
                    return "ABORT"
            return list(path.items())[0][0]
        else: #all tasks have been done
            self.glob_msg.pub_state("Going to tasks ended")
            return "Tasks Ended"  

class GNSS1(smach.State): #State for GNSS1
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection", "ABORT"],
                            input_keys = ["rem_loc_dict"],
                            output_keys = ["prev_loc", "aborted_state"])
        self.glob_msg = None
        
    def set_msg(self, glob_msg: GLOB_MSGS):
        self.glob_msg = glob_msg
        
    def execute(self, userdata):
        self.glob_msg.pub_state("Performing GNSS 1")        
        current_location_data = self.glob_msg.get_pose()

        current_distance = ((current_location_data.pose.position.x - userdata.rem_loc_dict["GNSS1"][0])**2 + 
                            (current_location_data.pose.position.y - userdata.rem_loc_dict["GNSS1"][1])**2)**(1/2) #Comparing how far we are from the target location
        
        if self.glob_msg.abort_check:
            self.glob_msg.pub_state("Aborting for state GNSS1")
            userdata.aborted_state = "GNSS1"
            return "ABORT"
        
        if current_distance < 2: #Determining whether we are in the correct location
            self.glob_msg.pub_state("GNSS1 reached, successful cruise")
            self.glob_msg.pub_state("Goal Point Reached: GNSS1")
            self.glob_msg.pub_led_light("mission done")
            rospy.sleep(3)
            self.glob_msg.pub_led_light("auto")
            
        else:
            self.glob_msg.pub_state("Failed to reach GNSS1 location")
            if self.glob_msg.abort_check:
                self.glob_msg.pub_state("Aborting for state GNSS1")
                userdata.aborted_state = "GNSS1"
                return "ABORT"
            pass
            
        userdata.prev_loc = "GNSS1"
        userdata.rem_loc_dict.pop(self.__class__.__name__) #Removing state from location list

        while(self.glob_msg.get_next_task_check() is not True):
            self.glob_msg.pub_state("Waiting for Next Task Button")
            rospy.sleep(1)
        self.glob_msg.next_task_check = False
        self.glob_msg.pub_led_light("auto")
        return "Location Selection"
        
        
class GNSS2(smach.State): #State for GNSS1
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection", "ABORT"],
                            input_keys = ["rem_loc_dict"],
                            output_keys = ["prev_loc", "aborted_state"])
        self.glob_msg = None
        
    def set_msg(self, glob_msg: GLOB_MSGS):
        self.glob_msg = glob_msg
        
    def execute(self, userdata):
        self.glob_msg.pub_state("Performing GNSS 2")
        
        current_location_data = self.glob_msg.get_pose()
        current_distance = ((current_location_data.pose.position.x - userdata.rem_loc_dict["GNSS2"][0])**2 + 
                            (current_location_data.pose.position.y - userdata.rem_loc_dict["GNSS2"][1])**2)**(1/2) #Comparing how far we are from the target location

        if self.glob_msg.abort_check:
            self.glob_msg.pub_state("Aborting for state GNSS2")
            userdata.aborted_state = "GNSS2"
            return "ABORT"
        
        if current_distance < 2:  #Determining whether we are in the correct location
            self.glob_msg.pub_state("GNSS2 reached, successful cruise")
            self.glob_msg.pub_state("Goal Point Reached: GNSS2")
            self.glob_msg.pub_led_light("mission done")
            # rospy.sleep(3)
            # self.glob_msg.pub_led_light("auto")
            
        else:
            self.glob_msg.pub_state("Failed to reach GNSS2 location")
            if self.glob_msg.abort_check:
                
                self.glob_msg.pub_state("Aborting for state GNSS2")
                userdata.aborted_state = "GNSS2"
                return "ABORT"
            pass
        #else:
        #print("Failed cruise")
        userdata.prev_loc = "GNSS2"
        userdata.rem_loc_dict.pop(self.__class__.__name__) #Removing state from location list

        while(self.glob_msg.get_next_task_check() is not True):
            self.glob_msg.pub_state("Waiting for Next Task Button")
            rospy.sleep(1)
        self.glob_msg.next_task_check = False
        self.glob_msg.pub_led_light("auto")
        return "Location Selection"

class AR1(smach.State): #State for AR1
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection", "ABORT"],
                            input_keys = ["rem_loc_dict"],
                            output_keys = ["prev_loc", "aborted_state"])
        self.glob_msg = None
        self.aruco_found = False
        
    def set_msg(self, glob_msg: GLOB_MSGS):
        self.glob_msg = glob_msg

    def aruco_callback(self, msg):
        self.aruco_found = msg.data

    def execute(self, userdata):
        self.glob_msg.pub_state("Performing AR1 Search")

        current_location_data = self.glob_msg.get_pose()

        current_distance = ((current_location_data.pose.position.x - userdata.rem_loc_dict["AR1"][0])**2 +  #Comparing how far we are from the target location
                            (current_location_data.pose.position.y - userdata.rem_loc_dict["AR1"][1])**2)**(1/2)
        
        if self.glob_msg.abort_check:
                self.glob_msg.pub_state("Aborting for state AR1")
                userdata.aborted_state = "AR1"
                return "ABORT"

        if current_distance < 5:
            # print("Successful cruise")
            self.glob_msg.pub_state("Reached AR1 GNSS")
            self.glob_msg.pub_state_name("AR1") 

            #ar_detector = ar_detection_node.ARucoTagDetectionNode() #calls the detection node
            
            if not self.glob_msg.done_early: #If the done early button is pressed, we will not do the grid search
                gs = sm_grid_search.GridSearch(sm_config.get("AR_grid_search_w"), sm_config.get("AR_grid_search_h"), sm_config.get("AR_grid_search_tol"), userdata.rem_loc_dict["AR1"][0], userdata.rem_loc_dict["AR1"][1])  #Creates an instance of the grid search class
                targets = gs.square_target() #Generates multiple points for grid search
                gs_traversal_object = sm_grid_search.GS_Traversal(sm_config.get("GS_Traversal_lin_vel"), sm_config.get("GS_Traversal_ang_vel"), targets, "AR1") #Starts grid search traversal
                rospy.Subscriber("aruco_found", Bool, self.aruco_callback) #Subscribes to aruco found to determine whether its found or not
                self.glob_msg.pub_state("Starting AR1 grid search")
                ar_in_correct_loc = gs_traversal_object.navigate() #Navigates to the generated grid search targets
                print("ar in correct loc", ar_in_correct_loc)
                self.glob_msg.pub_state("End of AR1 grid search")
                if self.glob_msg.abort_check:
                        self.glob_msg.pub_state("Aborting for state AR1")
                        userdata.aborted_state = "AR1"
                        self.glob_msg.pub_state_name("")
                        return "ABORT"
                
                if ar_in_correct_loc:
                    self.glob_msg.pub_state("Grid Search did find AR1") #Will publish the messages afterwards but there are topics to publish when detected 
                    # if ar_in_correct_loc:
                    self.glob_msg.pub_state("Close enough to AR1") 
                    self.glob_msg.pub_state("Goal Point Reached: AR1")
                    self.glob_msg.pub_led_light("mission done")
                    # rospy.sleep(3)
                    # self.glob_msg.pub_led_light("auto")
                    
                else:
                    self.glob_msg.pub_state("Grid Search did not find AR1")
                    if self.glob_msg.abort_check:
                        self.glob_msg.pub_state("Aborting for state AR1")
                        userdata.aborted_state = "AR1"
                        self.glob_msg.pub_state_name("")
                        return "ABORT"
        
                self.glob_msg.pub_state_name("")

            else: 
                self.glob_msg.pub_state("Done early do skipping grid search")
                self.glob_msg.pub_state("Goal Point Reached: AR1")
                self.glob_msg.pub_led_light("mission done")
                self.done_early=False
                
                if self.glob_msg.abort_check:
                    self.glob_msg.pub_state("Aborting for state AR1")
                    userdata.aborted_state = "AR1"
                    return "ABORT"
        else: # did not do straight line properly 
            self.glob_msg.pub_state("Did not reach AR2 GNSS")
            if self.glob_msg.abort_check:
                self.glob_msg.pub_state("Aborting for state AR2")
                userdata.aborted_state = "AR2"
                return "ABORT"
            
        userdata.prev_loc = "AR1"
        userdata.rem_loc_dict.pop(self.__class__.__name__) #remove state from location list

        while(self.glob_msg.get_next_task_check() is not True):
            self.glob_msg.pub_state("Waiting for Next Task Button")
            rospy.sleep(1)
        self.glob_msg.next_task_check = False
        self.glob_msg.pub_led_light("auto")
        return "Location Selection"
                    

class AR2(smach.State): #State for AR2
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection", "ABORT"],
                            input_keys = ["rem_loc_dict"],
                            output_keys = ["prev_loc", "aborted_state"])
        self.glob_msg = None
        self.aruco_found = False 
        
    def set_msg(self, glob_msg: GLOB_MSGS):
        self.glob_msg = glob_msg      

    def aruco_callback(self, msg):
        self.aruco_found = msg.data 
        
    def execute(self, userdata):
        self.glob_msg.pub_state("Performing AR2 Search")

        current_location_data = self.glob_msg.get_pose()
        current_distance = ((current_location_data.pose.position.x - userdata.rem_loc_dict["AR2"][0])**2 + 
                            (current_location_data.pose.position.y - userdata.rem_loc_dict["AR2"][1])**2)**(1/2)
        
        if self.glob_msg.abort_check:
            self.glob_msg.pub_state("Aborting for state AR2")
            userdata.aborted_state = "AR2"
            return "ABORT"

        if current_distance < 5:
            # print("Successful cruise")
            self.glob_msg.pub_state("Reached AR2 GNSS")
            self.glob_msg.pub_state_name("AR2") 
            if not self.glob_msg.done_early:
            #ar_detector = ar_detection_node.ARucoTagDetectionNode() #calls the detection node
                gs = sm_grid_search.GridSearch(sm_config.get("AR_grid_search_w"), sm_config.get("AR_grid_search_h"), sm_config.get("AR_grid_search_tol"), userdata.rem_loc_dict["AR2"][0], userdata.rem_loc_dict["AR2"][1])  # define multiple target points here: cartesian
                print(sm_config.get("AR_grid_search_w"))
                targets = gs.square_target() #generates multiple grid search targets 
            
                gs_traversal_object = sm_grid_search.GS_Traversal(sm_config.get("GS_Traversal_lin_vel"), sm_config.get("GS_Traversal_ang_vel"), targets, "AR2")
                rospy.Subscriber("aruco_found", Bool, self.aruco_callback)
                self.glob_msg.pub_state("Starting AR2 grid search")
                ar_in_correct_loc = gs_traversal_object.navigate() #Navigates to the grid search targets
                self.glob_msg.pub_state("End of AR2 grid search")
                if self.glob_msg.abort_check:
                        self.glob_msg.pub_state("Aborting for state AR2")
                        userdata.aborted_state = "AR2"
                        self.glob_msg.pub_state_name("")
                        return "ABORT"
                
                
                if ar_in_correct_loc:
                    self.glob_msg.pub_state("Grid Search did find AR2") #Will publish the messages afterwards but there are topics to publish when detected 
                    # if ar_in_correct_loc:
                    self.glob_msg.pub_state("Close enough to AR2") 
                    self.glob_msg.pub_state("Goal Point Reached: AR2")
                    self.glob_msg.pub_led_light("mission done")
                    # rospy.sleep(3)
                    # self.glob_msg.pub_led_light("auto")
                    
                else:
                    self.glob_msg.pub_state("Grid Search did not find AR2")
                    if self.glob_msg.abort_check:
                        self.glob_msg.pub_state("Aborting for state AR2")
                        userdata.aborted_state = "AR2"
                        self.glob_msg.pub_state_name("")
                        return "ABORT"
                self.glob_msg.pub_state_name("")

            else:
                self.glob_msg.pub_state("Done early do skipping grid search")
                self.glob_msg.pub_state("Goal Point Reached: AR1")
                self.glob_msg.pub_led_light("mission done")
                self.done_early=False
                    
                if self.glob_msg.abort_check:
                    self.glob_msg.pub_state("Aborting for state AR1")
                    userdata.aborted_state = "AR1"
                    return "ABORT"
        else:
            self.glob_msg.pub_state("Did not reach AR2 GNSS")
            if self.glob_msg.abort_check:
                self.glob_msg.pub_state("Aborting for state AR2")
                userdata.aborted_state = "AR2"
                return "ABORT"
            
        userdata.prev_loc = "AR2"
        userdata.rem_loc_dict.pop(self.__class__.__name__) #remove state from location list

        while(self.glob_msg.get_next_task_check() is not True):
            self.glob_msg.pub_state("Waiting for Next Task Button")
            rospy.sleep(1)
        self.glob_msg.next_task_check = False
        self.glob_msg.pub_led_light("auto")
        return "Location Selection"

class AR3(smach.State): #State for AR3
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection", "ABORT"],
                            input_keys = ["rem_loc_dict"],
                            output_keys = ["prev_loc", "aborted_state"])
        self.glob_msg = None
        self.aruco_found = False
        
    def set_msg(self, glob_msg: GLOB_MSGS):
        self.glob_msg = glob_msg   

    def aruco_callback(self, msg):
        self.aruco_found = msg.data

    def execute(self, userdata):
        self.glob_msg.pub_state("Performing AR3 Search")

        current_location_data = self.glob_msg.get_pose()
        current_distance = ((current_location_data.pose.position.x - userdata.rem_loc_dict["AR3"][0])**2 + 
                            (current_location_data.pose.position.y - userdata.rem_loc_dict["AR3"][1])**2)**(1/2)
        
        if self.glob_msg.abort_check:
            self.glob_msg.pub_state("Aborting for state AR3")
            userdata.aborted_state = "AR3"
            return "ABORT"

        if current_distance < 5:
            # print("Successful cruise")
            self.glob_msg.pub_state("Reached AR3 GNSS")
            self.glob_msg.pub_state_name("AR3") 
            if not self.glob_msg.done_early:
                #ar_detector = ar_detection_node.ARucoTagDetectionNode() #calls the detection node
                gs =  astar_obstacle_avoidance_grid_search.GridSearch(sm_config.get("AR_grid_search_w"), sm_config.get("AR_grid_search_h"), sm_config.get("AR_grid_search_tol"), userdata.rem_loc_dict["AR3"][0], userdata.rem_loc_dict["AR3"][1])  # define multiple target points here: cartesian
                gs =  astar_obstacle_avoidance_grid_search.GridSearch(sm_config.get("AR_grid_search_w"), sm_config.get("AR_grid_search_h"), sm_config.get("AR_grid_search_tol"), userdata.rem_loc_dict["AR3"][0], userdata.rem_loc_dict["AR3"][1])  # define multiple target points here: cartesian
                targets = gs.square_target() #generates multiple targets 
                gs_traversal_object = astar_obstacle_avoidance_grid_search.AstarObstacleAvoidance_GS_Traversal(sm_config.get("GS_Traversal_lin_vel"), sm_config.get("GS_Traversal_ang_vel"), targets, "AR3")
                rospy.Subscriber("aruco_found", Bool, self.aruco_callback)
                self.glob_msg.pub_state("Starting AR3 grid search")
                ar_in_correct_loc = gs_traversal_object.navigate() #Navigates to the grid search targets
                self.glob_msg.pub_state("End of AR3 grid search")
                if self.glob_msg.abort_check:
                        self.glob_msg.pub_state("Aborting for state AR3")
                        userdata.aborted_state = "AR3"
                        self.glob_msg.pub_state_name("")
                        return "ABORT"
                
                if self.aruco_found:
                    self.glob_msg.pub_state("Grid Search did find AR3") #Will publish the messages afterwards but there are topics to publish when detected 
                    # if ar_in_correct_loc:
                    #self.glgob_msg.pub_led_light("auto")
                        
                else:
                    self.glob_msg.pub_state("Grid Search did not find AR3")
                    if self.glob_msg.abort_check:
                        self.glob_msg.pub_state("Aborting for state AR3")
                        userdata.aborted_state = "AR3"
                        self.glob_msg.pub_state_name("")
                        return "ABORT"                
                
                self.glob_msg.pub_state_name("")
                
            else:
                self.glob_msg.pub_state("Done early do skipping grid search")
                self.glob_msg.pub_state("Goal Point Reached: AR1")
                self.glob_msg.pub_led_light("mission done")
                self.done_early=False
                    
                if self.glob_msg.abort_check:
                    self.glob_msg.pub_state("Aborting for state AR1")
                    userdata.aborted_state = "AR1"
                    return "ABORT"
                    
        else:
            self.glob_msg.pub_state("Did not reach AR3 GNSS")
            if self.glob_msg.abort_check:
                self.glob_msg.pub_state("Aborting for state AR3")
                userdata.aborted_state = "AR3"
                return "ABORT"
            # print("Failed to reach the location")
        userdata.prev_loc = "AR3"
        userdata.rem_loc_dict.pop(self.__class__.__name__) #remove state from location list

        while(self.glob_msg.get_next_task_check() is not True):
            self.glob_msg.pub_state("Waiting for Next Task Button")
            rospy.sleep(1)
        self.glob_msg.next_task_check = False
        self.glob_msg.pub_led_light("auto")
        return "Location Selection"

class OBJ1(smach.State): #State for mallet
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection", "ABORT"],
                            input_keys = ["rem_loc_dict"],
                            output_keys = ["prev_loc", "aborted_state"])
        self.glob_msg = None
        self.mallet_found = False
        self.waterbottle_found = False
        
    def set_msg(self, glob_msg: GLOB_MSGS):
        self.glob_msg = glob_msg    
    
    def mallet_callback(self, msg):
        self.mallet_found = msg.data

    def waterbottle_callback(self, msg):
        self.waterbottle_found = msg.data

    def execute(self, userdata):
        self.glob_msg.pub_state("Performing Object1 Search")

        current_location_data = self.glob_msg.get_pose()
        current_distance = ((current_location_data.pose.position.x - userdata.rem_loc_dict["OBJ1"][0])**2 + 
                            (current_location_data.pose.position.y - userdata.rem_loc_dict["OBJ1"][1])**2)**(1/2)
        
        if self.glob_msg.abort_check:
            self.glob_msg.pub_state("Aborting for state OBJ1")
            userdata.aborted_state = "OBJ1"
            return "ABORT"

        if current_distance < 5:
            # print("Successful cruise")
            self.glob_msg.pub_state("Reached Object1 GNSS")
            self.glob_msg.pub_state_name("OBJ1")
            if self.glob_msg.abort_check:
                gs = sm_grid_search.GridSearch(sm_config.get("OBJ_grid_search_w"), sm_config.get("OBJ_grid_search_h"), sm_config.get("OBJ_grid_search_tol"), userdata.rem_loc_dict["OBJ1"][0], userdata.rem_loc_dict["OBJ1"][1])  # define multiple target points here: cartesian
                targets = gs.square_target() #Generates grid search targets
                gs_traversal_object = sm_grid_search.GS_Traversal(sm_config.get("GS_Traversal_lin_vel"), sm_config.get("GS_Traversal_ang_vel"), targets, "OBJ1")
                rospy.Subscriber("mallet_detected", Bool, self.mallet_callback)
                rospy.Subscriber("waterbottle_detected", Bool, self.waterbottle_callback)

                self.glob_msg.pub_state("Starting OBJ1 grid search")
                obj1_in_correct_loc = gs_traversal_object.navigate() #Navigates to the grid search targets
                self.glob_msg.pub_state("Starting OBJ1 grid search")
                if self.glob_msg.abort_check:
                    self.glob_msg.pub_state("Aborting for state OBJ1")
                    userdata.aborted_state = "OBJ1"
                    self.glob_msg.pub_state_name("")
                    return "ABORT"     

                if self.mallet_found or self.waterbottle_found:
                    if self.mallet_found:
                        self.glob_msg.pub_state("Grid Search did find Mallet")
                    elif self.waterbottle_found:
                        self.glob_msg.pub_state("Grid Search did find Waterbottle")
                    if obj1_in_correct_loc:
                        self.glob_msg.pub_state("Close enough to OBJ1") 
                        self.glob_msg.pub_state("Goal Point Reached: OBJ1")
                        self.glob_msg.pub_led_light("mission done")
                        # rospy.sleep(3)
                        # self.glob_msg.pub_led_light("auto")

                    else:
                        self.glob_msg.pub_state("Grid Search did not find OBJ1")
                        if self.glob_msg.abort_check:
                            self.glob_msg.pub_state("Aborting for state OBJ1")
                            userdata.aborted_state = "OBJ1"
                            self.glob_msg.pub_state_name("")
                            return "ABORT"       
                
                self.glob_msg.pub_state_name("")
            else:
                self.glob_msg.pub_state("Done early do skipping grid search")
                self.glob_msg.pub_state("Goal Point Reached: AR1")
                self.glob_msg.pub_led_light("mission done")
                self.done_early=False
                    
                if self.glob_msg.abort_check:
                    self.glob_msg.pub_state("Aborting for state AR1")
                    userdata.aborted_state = "AR1"
                    return "ABORT"
        else:
            self.glob_msg.pub_state("Did not reach Object1 GNSS")
            if self.glob_msg.abort_check:
                self.glob_msg.pub_state("Aborting for state OBJ1")
                userdata.aborted_state = "OBJ1"
                self.glob_msg.pub_state_name("")
                return "ABORT"  
            
        userdata.prev_loc = "OBJ1"
        userdata.rem_loc_dict.pop(self.__class__.__name__) #remove state from location list

        while(self.glob_msg.get_next_task_check() is not True):
            self.glob_msg.pub_state("Waiting for Next Task Button")
            rospy.sleep(1)
        self.glob_msg.next_task_check = False
        self.glob_msg.pub_led_light("auto")
        return "Location Selection"
    
class OBJ2(smach.State): #State for waterbottle
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection", "ABORT"],
                            input_keys = ["rem_loc_dict"],
                            output_keys = ["prev_loc", "aborted_state"])
        self.glob_msg = None
        self.mallet_found = False
        self.waterbottle_found = False
        
    def set_msg(self, glob_msg: GLOB_MSGS):
        self.glob_msg = glob_msg    

    def mallet_callback(self, msg):
        self.mallet_found = msg.data

    def waterbottle_callback(self, msg):
        self.waterbottle_found = msg.data
    
    def execute(self, userdata): 
        self.glob_msg.pub_state("Performing OBject2 Search")
        current_location_data = self.glob_msg.get_pose()
        current_distance = ((current_location_data.pose.position.x - userdata.rem_loc_dict["OBJ2"][0])**2 + 
                            (current_location_data.pose.position.y - userdata.rem_loc_dict["OBJ2"][1])**2)**(1/2)
        
        if self.glob_msg.abort_check:
            self.glob_msg.pub_state("Aborting for state OBJ2")
            userdata.aborted_state = "OBJ2"
            return "ABORT"

        if current_distance < 5:
            # print("Successful cruise")
            self.glob_msg.pub_state("Reached Object2 GNSS")
            self.glob_msg.pub_state_name("OBJ2")
            if self.glob_msg.abort_check:
                gs = astar_obstacle_avoidance_grid_search.GridSearch(sm_config.get("OBJ_grid_search_w"), sm_config.get("OBJ_grid_search_h"), sm_config.get("OBJ_grid_search_tol"), userdata.rem_loc_dict["OBJ2"][0], userdata.rem_loc_dict["OBJ2"][1])  # define multiple target points here: cartesian
                gs = astar_obstacle_avoidance_grid_search.GridSearch(sm_config.get("OBJ_grid_search_w"), sm_config.get("OBJ_grid_search_h"), sm_config.get("OBJ_grid_search_tol"), userdata.rem_loc_dict["OBJ2"][0], userdata.rem_loc_dict["OBJ2"][1])  # define multiple target points here: cartesian
                targets = gs.square_target()
                gs_traversal_object = astar_obstacle_avoidance_grid_search.AstarObstacleAvoidance_GS_Traversal(sm_config.get("GS_Traversal_lin_vel"), sm_config.get("GS_Traversal_ang_vel"), targets, "OBJ2")
                gs_traversal_object = astar_obstacle_avoidance_grid_search.AstarObstacleAvoidance_GS_Traversal(sm_config.get("GS_Traversal_lin_vel"), sm_config.get("GS_Traversal_ang_vel"), targets, "OBJ2")
                rospy.Subscriber("mallet_detected", Bool, self.mallet_callback)
                rospy.Subscriber("waterbottle_detected", Bool, self.waterbottle_callback)

                self.glob_msg.pub_state("Starting OBJ2 grid search")
                obj1_in_correct_loc = gs_traversal_object.navigate() #Navigates to the grid search targets
                self.glob_msg.pub_state("End of OBJ2 grid search")
                if self.glob_msg.abort_check:
                    self.glob_msg.pub_state("Aborting for state OBJ2")
                    userdata.aborted_state = "OBJ2"
                    self.glob_msg.pub_state_name("")
                    return "ABORT"     

                if self.mallet_found or self.waterbottle_found:
                    if self.mallet_found:
                        self.glob_msg.pub_state("Grid Search did find Mallet")
                    elif self.waterbottle_found:
                        self.glob_msg.pub_state("Grid Search did find Waterbottle")
                    if obj1_in_correct_loc:
                        self.glob_msg.pub_state("Close enough to Object1") 
                        self.glob_msg.pub_state("Goal Point Reached: OBJ2")
                        self.glob_msg.pub_led_light("mission done")
                        # rospy.sleep(3)
                        # self.glob_msg.pub_led_light("auto")
                    else:
                        self.glob_msg.pub_state("Grid Search did not find Object1")
                        if self.glob_msg.abort_check:
                            self.glob_msg.pub_state("Aborting for state OBJ2")
                            userdata.aborted_state = "OBJ2"
                            self.glob_msg.pub_state_name("")
                            return "ABORT"    
                        
                
                self.glob_msg.pub_state_name("")
            else:
                self.glob_msg.pub_state("Done early do skipping grid search")
                self.glob_msg.pub_state("Goal Point Reached: AR1")
                self.glob_msg.pub_led_light("mission done")
                self.done_early=False
                    
                if self.glob_msg.abort_check:
                    self.glob_msg.pub_state("Aborting for state AR1")
                    userdata.aborted_state = "AR1"
                    return "ABORT"
        else:
            self.glob_msg.pub_state("Did not reach OBJ2 GNSS")
            if self.glob_msg.abort_check:
                self.glob_msg.pub_state("Aborting for state OBJ2")
                userdata.aborted_state = "OBJ2"
                self.glob_msg.pub_state_name("")
                return "ABORT"  
            
        userdata.prev_loc = "OBJ2"
        userdata.rem_loc_dict.pop(self.__class__.__name__) #remove state from location list

        while(self.glob_msg.get_next_task_check() is not True):
            self.glob_msg.pub_state("Waiting for Next Task Button")
            rospy.sleep(1)
        self.glob_msg.next_task_check = False
        self.glob_msg.pub_led_light("auto")
        return "Location Selection"
    
    
class ABORT(smach.State):  # Assuming it won't be called before we try to go to a task 
                           # (more specifically before the init state is called for the first time),
                           # what should we do in case of failure (unlikely)?
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["Location Selection"],
            input_keys=["prev_loc", "start_location", "aborted_state", "rem_loc_dict", "locations_dict"],
            output_keys=["rem_loc_dict"]
        )
        self.glob_msg = None

    def set_msg(self, glob_msg: GLOB_MSGS):
        self.glob_msg = glob_msg

    def execute(self, userdata):
        self.glob_msg.abort_check = False
        # Determine the location to return to
        if userdata.prev_loc == "start":
            self.glob_msg.pub_state("No previous location available; returning to start location.")
            return_location = (0,0)
            self.glob_msg.pub_state(f"Returning to start location {return_location}")
        else: 
            self.glob_msg.pub_state("Aborting back to the previous finished task location")
            return_location = userdata.locations_dict[userdata.prev_loc]
        self.glob_msg.abort_check = False
        self.glob_msg.pub_state(f"Aborting to location: {return_location} to {userdata.prev_loc}")
        if len(userdata.rem_loc_dict) == len(userdata.locations_dict):
            self.glob_msg.cartesian.pop(userdata.aborted_state)
        
        if len(userdata.rem_loc_dict) < len(userdata.locations_dict):
            userdata.rem_loc_dict.pop(userdata.aborted_state)

        try:
            # Perform straight-line traversal back to the determined location
            sla = StraightLineApproach(sm_config.get("straight_line_approach_lin_vel"), sm_config.get("straight_line_approach_ang_vel"), [return_location])
            #sla = StraightLineApproachNew(sm_config.get("straight_line_approach_lin_vel"), sm_config.get("straight_line_approach_ang_vel"), [return_location], userdata.prev_loc)
            self.glob_msg.pub_state("Navigating to Abort location")
            sla.navigate()
            rospy.sleep(2) #For waiting after the abort location is reached
            # Check final distance to the target location
            current_pose = self.glob_msg.get_pose()
            distance = math.sqrt(
                (current_pose.pose.position.x - return_location[0]) ** 2
                + (current_pose.pose.position.y - return_location[1]) ** 2
            )

            if distance < 2.0:  # Threshold for successful navigation
                self.glob_msg.pub_state("Successfully returned to the prev task location.")
                return "Location Selection"
            else:
                self.glob_msg.pub_state("Failed to reach the prev task location within threshold.")
                return "Location Selection"

        except rospy.ROSInterruptException as e:
            self.glob_msg.pub_state(f"ROS Interrupt Exception during abort: {e}")
            return "Location Selection"


class TasksEnded(smach.State):
    def __init__(self):
        smach.State.__init__(self)
        self.glob_msg = None

    def set_msg(self, glob_msg: GLOB_MSGS):
        self.glob_msg = glob_msg
    
    def execute(self, userdata):
        self.glob_msg.pub("All tasks finished yeeeyy!!!")

def main():
    rospy.init_node('RSX_Rover')
    #gui_status = rospy.Publisher('gui_status', String, queue_size=10) #where should be used?
    sm = smach.StateMachine(outcomes=["Tasks Ended"])
    glob_msg = GLOB_MSGS()
    init = InitializeAutonomousNavigation()
    locselect = LocationSelection()
    gnss1 = GNSS1()
    gnss2 = GNSS2()
    ar1 = AR1()
    ar2 = AR2()
    ar3 = AR3()
    obj1 = OBJ1()
    obj2 = OBJ2()
    tasks_ended = TasksEnded()
    abort = ABORT() 

    init.set_msg(glob_msg)
    locselect.set_msg(glob_msg)
    gnss1.set_msg(glob_msg)
    gnss2.set_msg(glob_msg)
    ar1.set_msg(glob_msg)
    ar2.set_msg(glob_msg)
    ar3.set_msg(glob_msg)
    obj1.set_msg(glob_msg)
    obj2.set_msg(glob_msg)
    tasks_ended.set_msg(glob_msg)
    abort.set_msg(glob_msg)

    with sm:
        smach.StateMachine.add(
            "Initialize Autonomous Navigation",
            init,
            transitions={"Tasks Execute" : "Tasks Execute"},
            remapping={"start_location" : "start_location"}
        )
        
        # Define the tasks execution sub-state machine
        sm_tasks_execute = smach.StateMachine(outcomes=["Tasks Ended"])
        sm_tasks_execute.userdata.locations_dict = {}
        sm_tasks_execute.userdata.start_location = None
        sm_tasks_execute.userdata.rem_loc_dict = {}
        sm_tasks_execute.userdata.prev_loc = "start"  # Initialize with the start location
        sm_tasks_execute.userdata.aborted_state = None

        with sm_tasks_execute:
            smach.StateMachine.add(
                "Location Selection",
                locselect,
                transitions={
                    state: state for state in (RUN_STATES + ["ABORT", "Tasks Ended"]) #changed this to only include states in RUN_STATES
                },
                remapping={
                    "rem_loc_dict": "rem_loc_dict",
                    "locations_dict": "locations_dict",
                    "prev_loc": "prev_loc",  # Add remapping for prev_loc
                    "start_location" : "start_location",
                    "aborted_state" : "aborted_state"
                }
            )
            
            if "GNSS1" in RUN_STATES:
                smach.StateMachine.add(
                    "GNSS1",
                    gnss1,
                    transitions={"Location Selection": "Location Selection", 
                                 "ABORT": "ABORT"},
                    remapping={
                        "rem_loc_dict": "rem_loc_dict",
                        "prev_loc": "prev_loc",  # Output the last completed location
                        "aborted_state" : "aborted_state"
                    }
                )
            
            if "GNSS2" in RUN_STATES:
                smach.StateMachine.add(
                    "GNSS2",
                    gnss2,
                    transitions={"Location Selection": "Location Selection",
                                 "ABORT" : "ABORT"},
                    remapping={
                        "rem_loc_dict": "rem_loc_dict",
                        "prev_loc": "prev_loc",
                        "aborted_state" : "aborted_state"
                    }
                )
            
            if "AR1" in RUN_STATES:
                smach.StateMachine.add(
                    "AR1",
                    ar1,
                    transitions={"Location Selection": "Location Selection",
                                 "ABORT" : "ABORT"},
                    remapping={
                        "rem_loc_dict": "rem_loc_dict",
                        "prev_loc": "prev_loc",
                        "aborted_state" : "aborted_state"
                    }
                )
            
            if "AR2" in RUN_STATES:
                smach.StateMachine.add(
                    "AR2",
                    ar2,
                    transitions={"Location Selection": "Location Selection",
                                 "ABORT" : "ABORT"},
                    remapping={
                        "rem_loc_dict": "rem_loc_dict",
                        "prev_loc": "prev_loc",
                        "aborted_state" : "aborted_state" 
                    }
                )
            
            if "AR3" in RUN_STATES:
                smach.StateMachine.add(
                    "AR3",
                    ar3,
                    transitions={"Location Selection": "Location Selection",
                                 "ABORT" : "ABORT"},
                    remapping={
                        "rem_loc_dict": "rem_loc_dict",
                        "prev_loc": "prev_loc",
                        "aborted_state" : "aborted_state"
                    }
                )
            
            if "OBJ1" in RUN_STATES:
                smach.StateMachine.add(
                    "OBJ1",
                    obj1,
                    transitions={"Location Selection": "Location Selection",
                                 "ABORT" : "ABORT"},
                    remapping={
                        "rem_loc_dict": "rem_loc_dict",
                        "prev_loc": "prev_loc",
                        "aborted_state" : "aborted_state"
                    }
                )
            
            if "OBJ2" in RUN_STATES:
                smach.StateMachine.add(
                    "OBJ2",
                    obj2,
                    transitions={"Location Selection": "Location Selection",
                                 "ABORT" : "ABORT"},
                    remapping={
                        "rem_loc_dict": "rem_loc_dict",
                        "prev_loc": "prev_loc",
                        "aborted_state" : "aborted_state"
                    }
                )
            
            smach.StateMachine.add(
                "ABORT",
                abort,
                transitions={
                    "Location Selection": "Location Selection" #What should do in the case where abort fails?
                },
                remapping= {
                    "prev_loc": "prev_loc",  # Use the previous location for abort
                    "start_location" : "start_location",
                    "rem_loc_dict": "rem_loc_dict",
                    "aborted_state" : "aborted_state",
                    "location_dict" : "location_dict"
                }
            )
        
        #don't check, always add this!    
        smach.StateMachine.add(
            "Tasks Execute",
            sm_tasks_execute,
            transitions={"Tasks Ended": "Tasks Ended"}
        )
        
        #dont check, always add!
        smach.StateMachine.add(
            "Task Ended",
            tasks_ended
        )
        

    sm.execute()
    
    


#Press initialize on GUI -- asks for the task locations and the current location automatically, then performs initialization
#In case of swithcing to manual control what should be done?
#Abort mode -- go back to the previous location, if its the first task it should go to the start location +
#When/How should the abort state be called? What should it do in case of failure?
#And if a task fails go to the previous location, try the next one, and put the other one at the end
#There should be a case that also considers going to the previous task teleop


if __name__ == "__main__":
    print("Starting the autonomous missions")
    main()