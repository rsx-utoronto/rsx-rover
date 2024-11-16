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

#Change this section according to potential get functions

ATag1_loc = ()
ATag2_loc = ()
ATag3_loc = () 
GNSS1_loc = ()
GNSS2_loc = ()
OBJ1_loc = ()
OBJ2_loc = ()

locations = {
    "GNSS1": GNSS1_loc,
    "GNSS2": GNSS2_loc,
    "AR1": ATag1_loc,
    "AR2": ATag2_loc,
    "AR3": ATag3_loc,
    "OBJ1": OBJ1_loc,
    "OBJ2": OBJ2_loc  
}

#Up till here

#0-1 scale, GNSS -> 1, 1
#AR --> 0.8 for without obstacles, 0.5 for obstacles
#Same for Objects


#Functions for now - Possible instances of classes

def shortest_path(start: tuple, locations: dict) -> (list[str]):

    start = (43.6532, 79.3832) # Toronto

    locations = {
    "GNSS1": (40.6892, -74.0445),   # Statue of Liberty
    "GNSS2": (48.8584, 2.2945),     # Eiffel Tower
    "AR1": (-33.8568, 151.2153),    # Sydney Opera House
    "AR2": (40.4319, 116.5704),     # Great Wall of China (Mutianyu)
    "AR3": (-22.9519, -43.2105),    # Christ the Redeemer
    "OBJ1": (27.1751, 78.0421),     # Taj Mahal
    "OBJ2": (41.8902, 12.4922),     # Colosseum
}
    OPmain(start, locations)

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

def rover_cruise(location) -> bool:
    pass

def grid_search_object():
    '''
    ach
    
    '''
    
def display_tag():
    """
    Function for displaying tag on GUI
    """
    #Body omitted for now- Possibly a class

    import serial
    import time

def get_current_location(port='/dev/ttyUSB0', baudrate=9600, timeout=1) -> tuple[float, float]:

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
        lat, lon = get_current_location()
        if lat is not None and lon is not None:
            print(f"Current Location: Latitude = {lat}, Longitude = {lon}")
        else:
            print("No GPS data available.")
        time.sleep(2)

    """
    """
    #Body omitted- possibly an instance of a class

def signal_green_led(): #happens for a few seconds and returns to red 
    """
    Function for signalling green led
    """
    #Body ommited for now- Possibly a class

def reinit():
    """
    Function for re-initializing the current coordinate
    """
    #Body ommited for now- Possibly a class


  
    
#*******************************************************

#Have to write code that checks which states are done and then passes to the undone states --< shortest path can track that +
#Is rover cruise a state or a class? +
#Can the states be simplified with parent classes? 


class InitializeAutonomousNavigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes = ["Location Selection"])
    
    def execute(self, userdata):
        rospy.loginfo("Initializing Autonomous Navigation")
        return "Location Selection"

        
class LocationSelection(smach.State): #goes through all states 
    def __init__(self):
        smach.State.__init__(self, outcomes = ["GNSS1", "GNSS2", "AR1", "AR2", "AR3", "OBJ1", "OBJ2"],
                             input_keys = ["rem_loc"],
                             output_keys = ["aimed_location"])
                            
    
    def execute(self, userdata):
        rospy.loginfo("Performing Location Search")
        path = userdata.rem_loc
        if path != []:
            rover_cruise(locations[path[0]])
            userdata.aimed_location = locations[path[0]]
            return path[0]
        else:
            return "Location Selection"  #In case of failure what should it do?   


class GNSS1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location", "rem_loc"])
        
    def execute(self, userdata):
        rospy.loginfo("Performing GNS 1")
        rover_cruise()
        if get_curr_loc() == userdata.aimed_location:
            rospy.loginfo("Successful cruise")
            signal_green_led()
        else:
            rospy.loginfo("Failed cruise")
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list
        return "Location Selection"
        
        
class GNSS2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location", "rem_loc"])
        curr_location = ()
        
    def execute(self, userdata):
        rospy.loginfo("Performing GNS 2")
        rover_cruise()
        if get_curr_loc() == userdata.aimed_location:
            rospy.loginfo("Successful cruise")
            signal_green_led()
        else:
            rospy.loginfo("Failed cruise")
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list
        return "Location Selection"
           
class AR1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location", "rem_loc"])
        curr_loc = get_curr_loc()
        
    def execute(self, userdata):
        rospy.loginfo("Performing ArıucoTag1 Search")
        state,loc=grid_search_aruco()
        if state == True:
            rospy.loginfo("Successful grid search")
            rover_cruise(loc)
            signal_green_led()
        else:
            rospy.loginfo("Failed grid search")
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list
        return "Location Selection"

class AR2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location", "rem_loc"])
        curr_location = ()
        
    def execute(self, userdata):
        rospy.loginfo("Performing ArıucoTag2 Search")
        state,loc=grid_search_aruco()
        if state == True:
            rospy.loginfo("Successful grid search")
            rover_cruise(loc)
            signal_green_led()
        else:
            rospy.loginfo("Failed grid search")
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list
        return "Location Selection"

class AR3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location", "rem_loc"])
        curr_location = ()
        
    def execute(self, userdata):
        rospy.loginfo("Performing ArıucoTag3 Search")
        state,loc=grid_search_aruco()
        if state == True:
            rospy.loginfo("Successful grid search")
            rover_cruise(loc)
            signal_green_led()
        else:
            rospy.loginfo("Failed grid search")
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list
        return "Location Selection"

class OBJ1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location", "rem_loc"])
        curr_location = ()
    
    def execute(self, userdata):
        rospy.loginfo("Performing ObjectNav 1")
        state,loc=grid_search_object()
        if state == True:
            rospy.loginfo("Successful grid search")
            signal_green_led()
        else:
            rospy.loginfo("Failed grid search")
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list
        return "Location Selection"
    
class OBJ2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location", "rem_loc"])
        curr_location = ()
    
    def execute(self, userdata):
        rospy.loginfo("Performing ObjectNav 2")
        state,loc=grid_search_object()
        if state == True:
            rospy.loginfo("Successful grid search")
            signal_green_led()
        else:
            rospy.loginfo("Failed grid search")
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list
        return "Location Selection" 

class TasksEnded(smach.State):
    def __init__(self):
        smach.State.__init__(self)
    
    def execute(self, userdata):
        rospy.loginfo("End of tasks")


def main():

    sm = smach.StateMachine(outcomes = ["Tasks Ended"])
    sm.userdata.locations_list = shortest_path(get_curr_loc, locations)
    sm.userdata.rem_loc = ()
    sm.userdata.aimed_location = ()

    with sm:
        smach.StateMachine.add("Initialize Autonomous Navigation", InitializeAutonomousNavigation(),
                               transitions = {"Tasks Execute": "Tasks Execute"})
        
        sm_tasks_execute = smach.StateMachine(outcomes = ["Tasks Ended"])

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

if __name__ == "__main__":
    t = time.time()
    while (time.time()-t) < 10:
        rospy.loginfo("Passing time until Autonomous Initialization")
        pass
    main()