#!/usr/bin/python3
"""
Code for the state machine

"""
import rospy
import smach
import smach_ros
import time 
import math
from optimal_path import OPmain
from thomas_grid_search import thomasgrid
from ar_detection_node import ARucoTagDetectionNode  
from aruco_homing import Aimer

#Change this section according to potential get functions


#Should be implemented with the gui so that the locations written would be here
locations = {
    "start": (43.6532, 79.3832), # Toronto
    "GNSS1": (40.6892, -74.0445),   # Statue of Liberty
    "GNSS2": (48.8584, 2.2945),     # Eiffel Tower
    "AR1": (-33.8568, 151.2153),    # Sydney Opera House
    "AR2": (40.4319, 116.5704),     # Great Wall of China (Mutianyu)
    "AR3": (-22.9519, -43.2105),    # Christ the Redeemer
    "OBJ1": (27.1751, 78.0421),     # Taj Mahal
    "OBJ2": (41.8902, 12.4922),     # Colosseum
}

#Up till here

#0-1 scale, GNSS -> 1, 1
#AR --> 0.8 for without obstacles, 0.5 for obstacles
#Same for Objects


#Functions for now - Possible instances of classes

#grid search should be runing for these, should stop when it doesn't detect anything after completeing the circle
#ar_detection_node : Looks for AR tags, if it doesn't detect it will ru, if it does detect it will print 
#the detected aruco tag bunch of times, the moment that it detects something it should call aruco_homing
#thomas_grid_search will return a bool value (both files are running at the same time)

#For rover_cruise call the staright_line_approach_kp 

#object_subscriber_node.py for object detection 


def shortest_path(start: str, locations: dict) -> list:
    return OPmain(start, locations)

def grid_search_aruco(): #should return a True or False,  #needs to display tag!
    """
    Function for grid search
    """
    # thomas grid needs to be switched out to a new program using straight line traversal kp
    # to do grid search (ie. pass in cartesian oordinates that would make up the grid) since
    # thomas grid uses... timing :')
    tg = thomasgrid.move()
    
    # run the grid search w ar_detection_node 
    
    # if ar_detection_node function "is_found" returns true at any point 
    return (True, (0,0))
    

def signal_red_led():
    """
    Function for signalling red led
    """
    #Body ommited for now- Possibly a class

def rover_cruise(location) -> bool:
    #straight_line_approach_kp 
    return True

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

'''
def get_curr_loc(port='/dev/ttyUSB0', baudrate=9600, timeout=1) -> tuple[float, float]:

    try:
        # Open the Serial Port
        with serial.Serial(port, baudrate=baudrate, timeout=timoemout) as gps_serial:
            while True:
                # Read a line of data from the GPS Module
                gps_data = gps.serial.readline().decode('utf-8', errors='ignore').strip()

                # Check for NMEA sentence that includes latitude and longitude
                if gps_data.startswith("$GPGGA"):
                    parts = gps_data.split(',')

                    if len(parts) > 5:
                        # Take out latitude and longitude 
                        raw_lat = float(parts[2])
                        lat_direction = parts[3]
                        raw_lon = float(parts[4])
                        lon_direction = parts[5]

                         # Convert raw latitude and longitude to degrees
                        latitude = int(raw_lat / 100) + (raw_lat % 100) / 60.0
                        longitude = int(raw_lon / 100) + (raw_lon % 100) / 60.0

                         # Adjust for N/S and E/W directions
                        if lat_direction == 'S':
                            latitude = -latitude
                        if lon_direction == 'W':
                            longitude = -longitude
                            return latitude, longitude
                            
    except serial.SerialException as e:
        print(f"Error: Could not read from serial port {port}. {e}")
        return None, None

        # Continuously print the rover's current location
if __name__ == "__main__":
    while True:
        lat, lon = get_curr_loc()
        if lat is not None and lon is not None:
            print(f"Current Location: Latitude = {lat}, Longitude = {lon}")
        else:
            print("No GPS data available.")
        time.sleep(2)

    """
    """
    #Body omitted- possibly an instance of a class
'''

def signal_green_led(): #happens for a few seconds and returns to red 
    """
    Function for signalling green led
    """
    #Body ommited for now- Possibly a class


    
#*******************************************************

#Have to write code that checks which states are done and then passes to the undone states --< shortest path can track that +
#Is rover cruise a state or a class? +
#Can the states be simplified with parent classes? 


class InitializeAutonomousNavigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes = ["Tasks Execute"])
    
    def execute(self, userdata):
        print("Initializing Autonomous Navigation")
        return "Tasks Execute"

        
class LocationSelection(smach.State): #goes through all states 
    def __init__(self):
        smach.State.__init__(self, outcomes = ["GNSS1", "GNSS2", "AR1", "AR2", "AR3", "OBJ1", "OBJ2", "Tasks Ended"],
                             input_keys = ["rem_loc", 'testing'],
                             output_keys = ["aimed_location"])
    
    def execute(self, userdata):
        print("Performing Location Search")
        path = userdata.rem_loc
        if path != []:
            rover_cruise(path[0])
            userdata.aimed_location = path[0]
            return path[0]
        else:
            return "Tasks Ended"  


class GNSS1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location", "rem_loc"])
        
    def execute(self, userdata):
        print("Performing GNS 1")
        rover_cruise(userdata.aimed_location)
        #if get_curr_loc() == userdata.aimed_location:
        print("Successful cruise")
        signal_green_led() # Should also be passed to the GUI 
        #else:
            #print("Failed cruise")
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list
        return "Location Selection"
        
        
class GNSS2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location", "rem_loc"])
        curr_location = ()
        
    def execute(self, userdata):
        print("Performing GNS 2")
        rover_cruise(userdata.aimed_location)
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
    
    with sm:
        smach.StateMachine.add("Initialize Autonomous Navigation", InitializeAutonomousNavigation(),
                               transitions = {"Tasks Execute": "Tasks Execute"})
        
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
    t = time.time()
    #while (time.time()-t) < 10:
    print("Passing time until Autonomous Initialization")
        #pass
    main()