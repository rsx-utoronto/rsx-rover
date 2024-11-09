"""
Code for the state machine

"""

import rospy
import smach
import smach_ros
import time 
import math

#Change this sectio according to potential get functions
ATag1_loc = ()
ATag2_loc = ()
ATag3_loc = () 
GNSS1_loc = ()
GNSS2_loc = ()
GOb1_loc = ()
GOb2_loc = ()

potential_locations = {"AR2": ATag1_loc, "AR2": ATag2_loc, "AR3": ATag3_loc, "GNSS1": GNSS1_loc, "GNSS2": GNSS2_loc, "GObject1":GOb1_loc, "GObject2": GOb2_loc}

#Up till here

#0-1 scale, GNSS -> 1, 1
#AR --> 0.8 for without obstacles, 0.5 for obstacles
#Same for Objects


#Functions for now - Possible instances of classes

def shortest_path(curr_loc: tuple, rem_loc: dict): 

    s_path = ["", float('inf')]
    
    #Get position of curr_loc tuple
    lat1 = curr_loc[0]
    lon1 = curr_loc[1]

    for loc in rem_loc:
        #Get position of loc
        lat2 = rem_loc[loc][0]
        lon2 = rem_loc[loc][1]
        print(lat1, lon1, lat2, lon2)
        # Convert latitude and longitude from degrees to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

        # Haversine formula
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)) 

        # Radius of Earth in kilometers
        r = 6371.0
        dist = r * c

        if dist < s_path[1]:
            s_path[0], s_path[1] = loc, dist

    return (s_path[0], rem_loc[s_path[0]])




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
                             outcomes = ["Location Selection"],
                             input_keys = ["curr_location"])
        
        curr_location = ()
    
    def execute(self, userdata):
        rospy.loginfo("Initializing Autonomous Navigation")
        return "Location Selection"

        
class LocationSelection(smach.State): #goes through all states 
    def __init__(self):
        smach.State.__init__(self, outcomes = ["GNSS1", "GNSS2", "AR1", "AR2", "AR3", "GObject1", "GObject2"],
                             output_keys = ["aimed_location"])

    
    def execute(self, userdata):
        rospy.loginfo("Performing Location Search")
        task_name, aimed_location = shortest_path()
        if aimed_location is not None:
            rover_cruise(aimed_location)
            return task_name
        else:
            return "Location Selection"  #In case of failure what should it do?   


class GNSS1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location"])
        curr_location = ()
        
    def execute(self, userdata):
        rospy.loginfo("Performing GNS 1")
        rover_cruise()
        if curr_location == aimed_location:
            rospy.loginfo("Successful cruise")
            signal_green_led()
        else:
            rospy.loginfo("Failed cruise")
        potential_locations.pop(self.__class__.__name__) #remove state from location list
        return "Location Selection"
        
        
class GNSS2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location"])
        curr_location = ()
        
    def execute(self, userdata):
        rospy.loginfo("Performing GNS 2")
        rover_cruise()
        if curr_location == aimed_location:
            rospy.loginfo("Successful cruise")
            signal_green_led()
        else:
            rospy.loginfo("Failed cruise")
        potential_locations.pop(self.__class__.__name__) #remove state from location list 
        return "Location Selection"
           
class AR1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location"])
        curr_location = ()
        
    def execute(self, userdata):
        rospy.loginfo("Performing ArıucoTag1 Search")
        state,loc=grid_search_aruco()
        if state == True:
            rospy.loginfo("Successful grid search")
            rover_cruise(loc)
            signal_green_led()
        else:
            rospy.loginfo("Failed grid search")
        potential_locations.pop(self.__class__.__name__) #remove state from location list
        return "Location Selection"

class AR2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location"])
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
        potential_locations.pop(self.__class__.__name__) #remove state from location list
        return "Location Selection"

class AR3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location"])
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
        potential_locations.pop(self.__class__.__name__) #remove state from location list
        return "Location Selection"

class GObject1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location"])
        curr_location = ()
    
    def execute(self, userdata):
        rospy.loginfo("Performing ObjectNav 1")
        state,loc=grid_search_object()
        if state == True:
            rospy.loginfo("Successful grid search")
            signal_green_led()
        else:
            rospy.loginfo("Failed grid search")
        potential_locations.pop(self.__class__.__name__) #remove state from location list
        return "Location Selection"
    
class GObject2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["aimed_location"])
        curr_location = ()
    
    def execute(self, userdata):
        rospy.loginfo("Performing ObjectNav 2")
        state,loc=grid_search_object()
        if state == True:
            rospy.loginfo("Successful grid search")
            signal_green_led()
        else:
            rospy.loginfo("Failed grid search")
        potential_locations.pop(self.__class__.__name__) #remove state from location list
        return "Location Selection" 

class TasksEnded(smach.State):
    def __init__(self):
        smach.State.__init__(self)
    
    def execute(self, userdata):
        rospy.loginfo("End of tasks")

def main():
    sm = smach.StateMachine(outcomes = ["Tasks Ended"]) 

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
                                        "GObject1": "GObject1",
                                        "GObject2": "GOBject2"})
        
            smach.StateMachine.add("GNSS1", GNSS1(),
                                transitions = {"Location Selection": "Location Selection"})
            
            smach.StateMachine.add("GNSS2", GNSS2(),
                                transitions = {"Location Selection": "Location Selection"})
            
            smach.StateMachine.add("AR1", AR1(),
                                transitions = {"Location Selection": "Location Selection"})
            
            smach.StateMachine.add("AR2", AR2(),
                                transitions = {"Location Selection": "Location Selection"})
            
            smach.StateMachine.add("AR3", AR3(),
                                transitions = {"Location Selection": "Location Selection"})
            
            smach.StateMachine.add("GObject1", GObject1(),
                                transitions = {"Location Selection": "Location Selection"})
                                
            smach.StateMachine.add("GObject2", GObject2(),
                                transitions = {"Location Selection": "Location Selection"})
        
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