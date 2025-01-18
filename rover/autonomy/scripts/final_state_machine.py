#!/usr/bin/python3
"""
Code for the state machine

"""
import rospy
import smach
import smach_ros
import time
import math
from math import atan2, pi, sin, cos
from rover.autonomy.scripts.optimal_path import OPmain
from thomas_grid_search import thomasgrid
from ar_detection_node import ARucoTagDetectionNode  
from aruco_homing import Aimer
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from sm_straight_line import StraightLineApproach
import sm_straight_line
import gps_conversion_functions as functions
import gps_to_pose as gps_to_pose
from std_msgs.msg import String

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

#lin_vel = 0.6, ang_vel = 0.3

def shortest_path(start: str, locations: dict) -> list:
    return OPmain(start, locations)

def grid_search_aruco(): #should return a True or False,  #needs to display tag!
    """
    Function for grid search
    """
    tg = thomasgrid.move()
    
    return (True, (0,0))
    

def signal_red_led():
    """
    Function for signalling red led
    """
    #Body ommited for now- Possibly a class

def rover_cruise(location):
    cartesianLocation = locations[location]
    StraightLineApproach(targets = cartesianLocation)

def grid_search_object():
    '''
    has to be completed
    '''
    return (True, (0,0))
    
def display_tag():
    """
    Function for displaying tag on GUI
    """
    #Body omitted for now- Possibly a class

    import serial
    import time


def signal_green_led(): #happens for a few seconds and returns to red 
    """
    Function for signalling green led
    """
    #Body ommited for now- Possibly a class


    
#*******************************************************


#Have to write code that checks which states are done and then passes to the undone states --> shortest path can track that +
#Is rover cruise a state or a class? +
#Can the states be simplified with parent classes? 


class InitializeAutonomousNavigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes = ["Tasks Execute"],
                             output_keys = ["cartesian"])
    
    def initialize(self):
        status_pub = rospy.Publisher('/status', String, queue_size = 10) #make this globval

        #Should be implemented with the gui so that the locations written would be here (write a subscribing function from the gui)
        location_data = rospy.Subscriber('/long_lat_goal_array', Float32MultiArray) 
        locations = {}
        location_name_list = ["start", "GNSS1", "GNSS2", "AR1", "AR2", "AR3", "OBJ1", "OBJ2"]
        for data in location_data:
            i = 0
            if data is not None:
                locations[location_name_list[i]] = data
                i +=1

        status_pub.publish("Changing GPS coordinates to cartesian") #at any print statement you can do this
        # locations [] -> carte_locations []
        cartesian = {}

        for el in locations: 
            distance = functions.getDistanceBetweenGPSCoordinates((locations["start"][0], locations["start"][1]), (el[0], el[1]))
            theta = functions.getHeadingBetweenGPSCoordinates(locations["start"][0], locations["start"][1], el[0], el[1])
            # since we measure from north y is r*cos(theta) and x is -r*sin(theta)
            x = -distance * sin(theta)
            y = distance * cos(theta)

            cartesian.update(el, (x,y))
    
    def execute(self, userdata):
        #Make an instance of the gps_to_pose class
        print("Initializing Autonomous Navigation")
        self.initialize()
        return "Tasks Execute"

        
class LocationSelection(smach.State): #goes through all states 
    def __init__(self):
        smach.State.__init__(self, outcomes = ["GNSS1", "GNSS2", "AR1", "AR2", "AR3", "OBJ1", "OBJ2", "Tasks Ended"],
                             input_keys = ["rem_loc", "cartesian"], #remloc is remaining locations array
                             output_keys = ["aimed_location"])
  
    def execute(self, userdata):
        print("Performing Location Search")
        path = userdata.rem_loc
        if path != []:
            try:
                sla = StraightLineApproach(1, 0.5, cartesian[path[0]]) 
            except rospy.ROSInterruptException:
                pass
            sla.navigate()
            # rover_cruise(path[0])
            # userdata.aimed_location = path[0]
            return path[0]
        else:
            return "Tasks Ended"  

class GNSS1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location", "rem_loc"])
        
    def execute(self, userdata):
        print("Performing GNS 1")
        # SUBSCRIBE TO MAP FRAME POSITION - WHERE ARE WE IN TERMS OF CARTESIAN (1-2 metres)
        # WHENEVER IT DETERMINES THAT WE'RE "DONE" - check euclidian distance (sqrt (delta x^2 delta y ^2))
        # Verify through testing if it is correct 

        # THIS IS WHAT WE NEED FOR GNSS1, GNSS2 ^^

        #For accessing the current location you can create a subscriber, look at the documentation online, get the x,y values, and 
        #convert them into euclidian distance

        #if get_curr_loc() == userdata.aimed_location:
        print("Successful cruise")
        signal_green_led() # Should also be passed to the GUI 
        #else:
            #print("Done cruise")
            #provide new point?
        
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list
        return "Location Selection"
        
        
class GNSS2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location", "rem_loc"])
        curr_location = ()
        
    def execute(self, userdata):
        print("Performing GNS 2")
        #if get_curr_loc() == userdata.aimed_location:
        print("Successful cruise")
        signal_green_led()
        #else:
        #print("Failed cruise")
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list
        return "Location Selection"
           
class AR1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location", "rem_loc"])
        #curr_loc = get_curr_loc()
        
    def execute(self, userdata):
        print("Performing ArucoTag1 Search")

        state,loc=grid_search_aruco() 
        if state == True:
            print("Successful grid search")
            rover_cruise(loc) #The aruco tag code is already going to cruise when it detects
            signal_green_led()
        else:
            print("Failed grid search")
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list
        return "Location Selection"

class AR2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location", "rem_loc"])
        #curr_location = get_curr_loc()
        
    def execute(self, userdata):
        print("Performing ArucoTag2 Search")
        state,loc = grid_search_aruco()
        if state == True:
            print("Successful grid search")
            rover_cruise(loc)
            signal_green_led()
        else:
            print("Failed grid search")
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list
        return "Location Selection"

class AR3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location", "rem_loc"])
        #curr_location = get_curr_loc()
        
    def execute(self, userdata):
        print("Performing ArucoTag3 Search")
        state,loc=grid_search_aruco()
        if state == True:
            print("Successful grid search")
            rover_cruise(loc)
            signal_green_led()
        else:
            print("Failed grid search")
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list
        return "Location Selection"

class OBJ1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location", "rem_loc"])
        #curr_location = get_curr_loc()
    
    def execute(self, userdata):
        print("Performing ObjectNav 1")
        state,loc = grid_search_object()
        if state == True:
            print("Successful grid search")
            signal_green_led()
        else:
            print("Failed grid search")
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list
        return "Location Selection"
    
class OBJ2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location", "rem_loc"])
        #get_curr_location = ()
    
    def execute(self, userdata):
        print("Performing ObjectNav 2")
        state,loc=grid_search_object()
        if state == True:
            print("Successful grid search")
            signal_green_led()
        else:
            print("Failed grid search")
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list
        return "Location Selection" 

class TasksEnded(smach.State):
    def __init__(self):
        smach.State.__init__(self)
    
    def execute(self, userdata):
        print("End of tasks")


def main():

    rospy.init_node('RSX_Rover')
    sm = smach.StateMachine(outcomes = ["Tasks Ended"])
    sm.cartesian = {}
    
    with sm:
        smach.StateMachine.add("Initialize Autonomous Navigation", InitializeAutonomousNavigation(),
                               transitions = {"Tasks Execute": "Tasks Execute"},
                               remapping = {"cartesian": "cartesian"})
        
        #Create a global variable that passes the cartesian list to location selection
        temp_locations_list = shortest_path('start', locations)
        temp_locations_list.remove('start')
        #Pass the path to the GUI
        sm_tasks_execute = smach.StateMachine(outcomes = ["Tasks Ended"])
        sm_tasks_execute.userdata.locations_list = temp_locations_list
        sm_tasks_execute.userdata.rem_loc = temp_locations_list.copy()
        sm_tasks_execute.userdata.aimed_location = ''


        with sm_tasks_execute:

            smach.StateMachine.add("Location Selection", LocationSelection(),
                        transitions = {"GNSS1": "GNSS1",
                                        "GNSS2": "GNSS2",
                                        "AR1": "AR1",
                                        "AR2": "AR2",
                                        "AR3": "AR3",
                                        "OBJ1": "OBJ1",
                                        "OBJ2": "OBJ2"},
                        remapping = {"rem_loc": "locations_list",
                                     "aimed_location": "aimed_location"})
        
            smach.StateMachine.add("GNSS1", GNSS1(),
                                transitions = {"Location Selection": "Location Selection"},
                                remapping = {"rem_loc": "locations_list",
                                     "aimed_location": "aimed_location"})
            
            smach.StateMachine.add("GNSS2", GNSS2(),
                                transitions = {"Location Selection": "Location Selection"},
                                remapping = {"rem_loc": "locations_list",
                                     "aimed_location": "aimed_location"})
            
            smach.StateMachine.add("AR1", AR1(),
                                transitions = {"Location Selection": "Location Selection"},
                                remapping = {"rem_loc": "locations_list",
                                     "aimed_location": "aimed_location"})
            
            smach.StateMachine.add("AR2", AR2(),
                                transitions = {"Location Selection": "Location Selection"},
                                remapping = {"rem_loc": "locations_list",
                                     "aimed_location": "aimed_location"})
            
            smach.StateMachine.add("AR3", AR3(),
                                transitions = {"Location Selection": "Location Selection"},
                                remapping = {"rem_loc": "locations_list",
                                     "aimed_location": "aimed_location"})
            
            smach.StateMachine.add("OBJ1", OBJ1(),
                                transitions = {"Location Selection": "Location Selection"},
                                remapping = {"rem_loc": "locations_list",
                                     "aimed_location": "aimed_location"})
                                
            smach.StateMachine.add("OBJ2", OBJ2(),
                                transitions = {"Location Selection": "Location Selection"},
                                remapping = {"rem_loc": "locations_list",
                                     "aimed_location": "aimed_location"})
        
        smach.StateMachine.add("Tasks Execute", sm_tasks_execute,
                               transitions = {"Tasks Ended": "Tasks Ended"})
        
        smach.StateMachine.add("Task Ended", TasksEnded())

    outcome = sm.execute()


#We don't need to wait on the GUI we can tell it to initialize
#Press initialize on GUI -- asks for the task locations and the current location automatically, then performs initialization
#In case of swithcing to manual control what should be done?
#Abort mode -- go back to the previous location, if its the first task it should go to the start location
#And if a task fails go to the previous location, try the next one, and put the other one at the end
#There should be a case that also considers going to the previous task teleop

if __name__ == "__main__":
    # run kat's code here
    InitializeAutonomousNavigation
    #t = time.time()
    #while (time.time()-t) < 10:
    print("Passing time until Autonomous Initialization")
        #pass
    main()