"""
Code for the state machine

"""

import rospy
import smach
import smach_ros
import time

#Change this sectio according to potential get functions
ATag1_loc = ()
ATag2_loc = ()
ATag3_loc = ()
GNSS1_loc = ()
GNSS2_loc = ()
GOb1_loc = ()
GOb2_loc = ()

potential_locations = [ATag1_loc, ATag2_loc, ATag3_loc, GNSS1_loc, GNSS2_loc, GOb1_loc, GOb2_loc]

#Up till here


t = time.time()
while (time.time()-t) < 10:
    rospy.loginfo("Passing time until Autonomous Inıtialization")
    pass

#Functions for now - Possible instances of classes

def shortest_path() -> tuple(int, int): 
    """
    Function for determining the potential location with the shortest distance
    """
    #calculations for each location

def grid_search_aruco() -> tuple(bool, tuple(int, int)): #should return a True or False,  #needs to display tag!
    """
    Function for grid search
    """
    #Body omitted for now-Possibly a class

def signal_red_led():
    """
    Function for signalling red led
    """
    #Body ommited for now- Possibly a class

def rover_cruise() -> bool:
    """
    Function for rover cruise
    """
    #Body omitted for now

def grid_search_object():
    '''
    ach
    
    '''
    

def display_tag():
    """
    Funciton for displaying tag on GUI
    """
    #Body omitted for now- Possibly a class

def get_curr_loc() -> tuple(int, int):
    """
    Function for getting current location
    """
    #Body omitted- possibly an instance of a class

def signal_green_led(): #happens for a few seconds and returns to red 
    """
    Function for signalling green led
    """
    #Body ommited for now- Possibly a class

#*******************************************************



class InitializeAutonomousNavigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes = ["Location Selection", "Re-Initialize"],
                             input_keys = ["curr_location"])
        
        curr_location = ()
    
    def execute(self, userdata):
        rospy.loginfo("Initializing Autonomous Navigation")
        if userdata.curr_loc is not None:
            return "Location Selection"
        else:
            return "Re-Initialize"
        
class LocationSelection(smach.State): #goes through all states 
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                             output_keys = ["aimed_location"])

    
    def execute(self, userdata):
        rospy.loginfo("Performing Location Search")
        aimed_location = shortest_path()
        if aimed_location is not None:
            return "Rover Cruise"
        else:
            return "Location Selection"  #In case of failure what should it do?     


class GNS1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location"])
        curr_location = ()
        
    def exectue(self, userdata):
        rospy.loginfo("Performing GNS 1")
        rover_cruise()
        if curr_location == aimed_location:
            rospy.loginfo("Successful cruise")
            signal_green_led()
        else:
            rospy.loginfo("Failed cruise")
        return "Location Selection"
        
        
class GNS2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location"])
        curr_location = ()
        
    def exectue(self, userdata):
        rospy.loginfo("Performing GNS 2")
        rover_cruise()
        if curr_location == aimed_location:
            rospy.loginfo("Successful cruise")
            signal_green_led()
        else:
            rospy.loginfo("Failed cruise")
        return "Location Selection"
           
class AR1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location"])
        curr_location = ()
        
    def exectue(self, userdata):
        rospy.loginfo("Performing ArıucoTag1 Search")
        state,loc=grid_search_aruco()
        if state == True:
            rospy.loginfo("Successful grid search")
            rover_cruise(loc)
            signal_green_led()
        else:
            rospy.loginfo("Failed grid search")
        return "Location Selection"

class AR2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location"])
        curr_location = ()
        
    def exectue(self, userdata):
        rospy.loginfo("Performing ArıucoTag2 Search")
        state,loc=grid_search_aruco()
        if state == True:
            rospy.loginfo("Successful grid search")
            rover_cruise(loc)
            signal_green_led()
        else:
            rospy.loginfo("Failed grid search")
        return "Location Selection"

class AR3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location"])
        curr_location = ()
        
    def exectue(self, userdata):
        rospy.loginfo("Performing ArıucoTag3 Search")
        state,loc=grid_search_aruco()
        if state == True:
            rospy.loginfo("Successful grid search")
            rover_cruise(loc)
            signal_green_led()
        else:
            rospy.loginfo("Failed grid search")
        return "Location Selection"

class GObject1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location"])
        curr_location = ()
    
    def exectue(self, userdata):
        rospy.loginfo("Performing ObjectNav 1")
        state,loc=grid_search_object()
        if state == True:
            rospy.loginfo("Successful grid search")
            signal_green_led()
        else:
            rospy.loginfo("Failed grid search")
        return "Location Selection"
    
class GObject2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location"])
        curr_location = ()
    
    def exectue(self, userdata):
        rospy.loginfo("Performing ObjectNav 2")
        state,loc=grid_search_object()
        if state == True:
            rospy.loginfo("Successful grid search")
            signal_green_led()
        else:
            rospy.loginfo("Failed grid search")
        return "Location Selection" 