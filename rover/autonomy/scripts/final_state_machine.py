#!/usr/bin/python3
"""
Code for the state machine

"""
import rospy
import object_subscriber_node
import smach
import time
import math
from optimal_path import OPmain
from thomas_grid_search import thomasgrid
import ar_detection_node  
from std_msgs.msg import Float32MultiArray, Bool, Float64MultiArray
from geometry_msgs.msg import Twist
from sm_straight_line import StraightLineApproach
import sm_straight_line
import gps_conversion_functions as functions
import gps_to_pose as gps_to_pose
import sm_grid_search
import ar_detection_node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


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

#Things Left To Do:
#-Testing
#-Signalling red/green light --> cases
#-Integration with GUI --> How exactly should it be done?
#-Abort state --> How/When should it be called? What should be done in case of failure
#-Teleop --> How/When should it be executed?


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

def signal_red_led():
    """
    Function for signalling red led
    """
    #Body ommited for now- Possibly a class
    
def signal_green_led(): #happens for a few seconds and returns to red 
    """
    Function for signalling green led
    """
    #Body ommited for now- Possibly a class

    
#*******************************************************

# should be able to access in all other classes 
class GLOB_MSGS:
    def __init__(self):
        self.pub = rospy.Publisher("gui_status", String, queue_size=10)
        self.sub = rospy.Subscriber("pose", PoseStamped, self.pose_callback)
        self.gui_loc = rospy.Subscriber('/long_lat_goal_array', Float32MultiArray, self.coord_callback) 
        self.locations = None
        
    def pose_callback(self, msg):
        self.pose = msg
        print(self.pose.header.stamp.secs, "in pose_callback")

    def get_pose(self):
        return self.pose

    def coord_callback(self, data): 
        location_data = data 

        locations = {}
        location_name_list = ["start", "GNSS1", "GNSS2", "AR1", "AR2", "AR3", "OBJ1", "OBJ2"]
        for data in location_data:
            i = 0
            if data is not None:
                locations[location_name_list[i]] = data
                i +=1
        
        self.locations = locations

    def pub_state(self, state):
        self.pub.publish(state)
    
    

class InitializeAutonomousNavigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes = ["Tasks Execute"],
                             output_keys = ["cartesian"])
        self.glob_msg = None
        
    def set_msg(self, glob_msg: GLOB_MSGS):
        self.glob_msg = glob_msg
    

    def initialize(self):
        
        #status_pub = rospy.Publisher('/status', String, queue_size = 10) #make this globval --> done+
        
        rospy.Subscriber("/long_lat_goal_array", Float32MultiArray, self.glob_msg.coord_callback) #gui location publisher
        while (self.glob_msg.locations is None and rospy.is_shutdown() == False):
            time.sleep(1)
            print("Waiting for locations")
        
        cartesian_path = shortest_path('start', self.glob_msg.locations) #code for generating the optimal path

        self.glob_msg.pub_state("Changing GPS coordinates to cartesian") #at any print statement you can do this +
        # locations [] -> carte_locations []

        for el in cartesian_path: 
            distance = functions.getDistanceBetweenGPSCoordinates((self.glob_msg.locations["start"][0], self.glob_msg.locations["start"][1]), (el[0], el[1]))
            theta = functions.getHeadingBetweenGPSCoordinates(self.glob_msg.locations["start"][0], self.glob_msg.locations["start"][1], el[0], el[1])
            # since we measure from north y is r*cos(theta) and x is -r*sin(theta)
            x = -distance * math.sin(theta)
            y = distance * math.cos(theta)

            self.cartesian.update(el, (x,y))
        
        gps_to_pose.GPSToPose(self.glob_msg.locations['start'], (0,0), (0,0))
        
    
    def execute(self, userdata):
        print("Initializing Autonomous Navigation")
        self.initialize()
        return "Tasks Execute"

        
class LocationSelection(smach.State): #goes through all states 
    def __init__(self):
        smach.State.__init__(self, outcomes = ["GNSS1", "GNSS2", "AR1", "AR2", "AR3", "OBJ1", "OBJ2", "Tasks Ended"],
                             input_keys = ["rem_loc", "locations_list_c"])
  
    def execute(self, userdata):
        print("Performing Location Search")
        path = userdata.rem_loc
        if path != []:
            try:
                sla = StraightLineApproach(0.6, 0.3, self.cartesian[path[0]]) 
            except rospy.ROSInterruptException:
                pass
            sla.navigate()
            return path[0]
        else:
            return "Tasks Ended"  

class GNSS1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["rem_loc"],
                            output_keys = ["prev_loc"])
        self.glob_msg = None
        
    def set_msg(self, glob_msg: GLOB_MSGS):
        self.glob_msg = glob_msg
        
    def execute(self, userdata):
        print("Performing GNS 1")
        self.glob_msg.pub_state("Performing GNSS 1")        
        current_location_data = self.glob_msg.get_pose()

        current_distance = ((current_location_data.pose.position.x - userdata.rem_loc[0][0])**2 + 
                            (current_location_data.pose.position.y - userdata.rem_loc[0][1])**2)**(1/2)

        if current_distance < 2:
            print("Successful cruise")
            self.glob_msg.pub_state("GNSS 1 reached")
            signal_green_led() # Should also be passed to the GUI 
        #else:
            #print("Done cruise")
            #provide new point?

        userdata.prev_loc = userdata.rem_loc[0]
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list

        return "Location Selection"
        
        
class GNSS2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["rem_loc"],
                            output_keys = ["prev_loc"])
        self.glob_msg = None
        
    def set_msg(self, glob_msg: GLOB_MSGS):
        self.glob_msg = glob_msg
        
    def execute(self, userdata):
        print("Performing GNS 2")
        self.glob_msg.pub_state("Performing GNSS 2")
        

        current_location_data = self.glob_msg.get_pose()

        current_distance = ((current_location_data.pose.position.x - userdata.rem_loc[0][0])**2 + 
                            (current_location_data.pose.position.y - userdata.rem_loc[0][1])**2)**(1/2)

        if current_distance < 2:
            print("Successful cruise")
            self.glob_msg.pub_state("GNSS 2 reached")
            signal_green_led()
        #else:
        #print("Failed cruise")
        userdata.prev_loc = userdata.rem_loc[0]
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list

        return "Location Selection"

class AR1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["rem_loc"],
                            output_keys = ["prev_loc"])
        self.glob_msg = None
        
    def set_msg(self, glob_msg: GLOB_MSGS):
        self.glob_msg = glob_msg

    def execute(self, userdata):
        self.glob_msg.pub_state("Performing AR1 Search")

        current_location_data = self.glob_msg.get_pose()

        current_distance = ((current_location_data.pose.position.x - userdata.rem_loc[0][0])**2 + 
                            (current_location_data.pose.position.y - userdata.rem_loc[0][1])**2)**(1/2)

        if current_distance < 2:
            # print("Successful cruise")
            self.glob_msg.pub_state("Reached AR1 GNSS")

            rospy.init_node('aruco_tag1_detector', anonymous=True)
            ar_detector = ar_detection_node.ARucoTagDetectionNode() #calls the detection node
            gs = sm_grid_search 
            gs_points = gs.GridSearch(10, 10, 1, userdata.rem_loc[0][0], userdata.rem_loc[0][1])  # define multiple target points here: cartesian
            targets = gs.square_target() #generates multiple targets 
            gs_traversal_object = sm_grid_search.GS_Traversal(0.6, 0.3, targets)

            aruco_found = rospy.Subscriber("aruco_found", Bool, ar_detector)
            ar_in_correct_loc = gs_traversal_object.navigate("AR1") #publishing messages?
            
            if aruco_found:
                self.glob_msg.pub_state("Grid Search did find AR1") #Will publish the messages afterwards but there are topics to publish when detected 
                if ar_in_correct_loc:
                    self.glob_msg.pub_state("Close enough to AR1") 
                    signal_green_led()

            else:
                self.glob_msg.pub_state("Grid Search did not find AR1")
        else:
            self.glob_msg.pub_state("Did not reach AR1 GNSS")
            # print("Failed to reach the location")
        userdata.prev_loc = userdata.rem_loc[0]
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list
        return "Location Selection"
                    

class AR2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["rem_loc"],
                            output_keys = ["prev_loc"])
        self.glob_msg = None
        
    def set_msg(self, glob_msg: GLOB_MSGS):
        self.glob_msg = glob_msg        
        
    def execute(self, userdata):
        self.glob_msg.pub_state("Performing AR2 Search")
        print("Performing AR2 Search")

        current_location_data = self.glob_msg.get_pose()
        current_distance = ((current_location_data.pose.position.x - userdata.rem_loc[0][0])**2 + 
                            (current_location_data.pose.position.y - userdata.rem_loc[0][1])**2)**(1/2)

        if current_distance < 2:
            # print("Successful cruise")
            self.glob_msg.pub_state("Reached AR2 GNSS")  

            rospy.init_node('aruco_tag2_detector', anonymous=True)
            ar_detector = ar_detection_node.ARucoTagDetectionNode() #calls the detection node
            gs = sm_grid_search 
            gs_points = gs.GridSearch(10, 10, 1, userdata.rem_loc[0][0], userdata.rem_loc[0][1])  # define multiple target points here: cartesian
            targets = gs.square_target() #generates multiple targets 
            gs_traversal_object = sm_grid_search.GS_Traversal(0.6, 0.3, targets)

            aruco_found = rospy.Subscriber("aruco_found", Bool, ar_detector)
            ar_in_correct_loc = gs_traversal_object.navigate("AR2") #publishing messages?
            
            if aruco_found:
                self.glob_msg.pub_state("Grid Search did find AR2")
                if ar_in_correct_loc:
                    self.glob_msg.pub_state("Close enough to AR2") 
                    signal_green_led()

                else:
                    self.glob_msg.pub_state("Grid Search did not find AR2")
        else:
            self.glob_msg.pub_state("Did not reach AR2 GNSS")
            # print("Failed to reach the location")
        userdata.prev_loc = userdata.rem_loc[0]
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list
        return "Location Selection"

class AR3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["rem_loc"],
                            output_keys = ["prev_loc"])
        self.glob_msg = None
        
    def set_msg(self, glob_msg: GLOB_MSGS):
        self.glob_msg = glob_msg    

    def execute(self, userdata):
        print("Performing AR3 Search")
        self.glob_msg.pub_state("Performing AR3 Search")

        current_location_data = self.glob_msg.get_pose()
        current_distance = ((current_location_data.pose.position.x - userdata.rem_loc[0][0])**2 + 
                            (current_location_data.pose.position.y - userdata.rem_loc[0][1])**2)**(1/2)

        if current_distance < 2:
            # print("Successful cruise")
            self.glob_msg.pub_state("Reached AR3 GNSS")

            rospy.init_node('aruco_tag3_detector', anonymous=True)
            ar_detector = ar_detection_node.ARucoTagDetectionNode() #calls the detection node
            gs = sm_grid_search 
            gs_points = gs.GridSearch(10, 10, 1, userdata.rem_loc[0][0], userdata.rem_loc[0][1])  # define multiple target points here: cartesian
            targets = gs.square_target() #generates multiple targets 
            gs_traversal_object = sm_grid_search.GS_Traversal(0.6, 0.3, targets)

            aruco_found = rospy.Subscriber("aruco_found", Bool, ar_detector)
            ar_in_correct_loc = gs_traversal_object.navigate("AR3") #publishing messages?
            
            if aruco_found:
                self.glob_msg.pub_state("Grid Search did find AR3")
                if ar_in_correct_loc:
                    self.glob_msg.pub_state("Close enough to AR3") 
                    signal_green_led()

                else:
                    self.glob_msg.pub_state("Grid Search did not find AR3")
        else:
            self.glob_msg.pub_state("Did not reach AR3 GNSS")
            # print("Failed to reach the location")
        userdata.prev_loc = userdata.rem_loc[0]
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list
        return "Location Selection"

class OBJ1(smach.State): #mallet
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["rem_loc"],
                            output_keys = ["prev_loc"])
        self.glob_msg = None
        
    def set_msg(self, glob_msg: GLOB_MSGS):
        self.glob_msg = glob_msg    

    def execute(self, userdata):
        self.glob_msg.pub_state("Performing Mallet Search")
        print("Performing Mallet Search")

        current_location_data = self.glob_msg.get_pose()
        current_distance = ((current_location_data.pose.position.x - userdata.rem_loc[0][0])**2 + 
                            (current_location_data.pose.position.y - userdata.rem_loc[0][1])**2)**(1/2)

        if current_distance < 2:
            # print("Successful cruise")
            self.glob_msg.pub_state("Reached Mallet GNSS")

            rospy.init_node('mallet_detector', anonymous=True) 
            object_detector = object_subscriber_node.ObjectDetectionNode()
            gs = sm_grid_search
            gs_points = gs.GridSearch(5, 5, 3, userdata.rem_loc[0][0], userdata.rem_loc[0][1])  # define multiple target points here: cartesian
            targets = gs.square_target()
            gs_traversal_object = sm_grid_search.GS_Traversal(0.6, 0.3, targets)
            mallet_detected = rospy.Subscriber("mallet_detected", Bool, queue_size = 1)
            obj1_in_correct_loc = gs_traversal_object.navigate("OBJ1") #publishing messages?

            if mallet_detected:
                self.glob_msg.pub_state("Grid Search did find Mallet")
                if obj1_in_correct_loc:
                    self.glob_msg.pub_state("Close enough to Mallet") 
                    signal_green_led()

                else:
                    self.glob_msg.pub_state("Grid Search did not find Mallet")
        else:
            self.glob_msg.pub_state("Did not reach Mallet GNSS")
        userdata.prev_loc = userdata.rem_loc[0]
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list
        return "Location Selection"
    
class OBJ2(smach.State): #waterbottle
    def __init__(self):
        smach.State.__init__(self, outcomes = ["Location Selection"],
                            input_keys = ["rem_loc"],
                            output_keys = ["prev_loc"])
        self.glob_msg = None
        
    def set_msg(self, glob_msg: GLOB_MSGS):
        self.glob_msg = glob_msg    

    def execute(self, userdata):
        self.glob_msg.pub_state("Performing Waterbottle Search")
        print("Performing Waterbottle Search")

        current_location_data = self.glob_msg.get_pose()
        current_distance = ((current_location_data.pose.position.x - userdata.rem_loc[0][0])**2 + 
                            (current_location_data.pose.position.y - userdata.rem_loc[0][1])**2)**(1/2)

        if current_distance < 2:
            # print("Successful cruise")
            self.glob_msg.pub_state("Reached Waterbottle GNSS")

            print("Performing Waterbottle Search")
            rospy.init_node('waterbottle_detector', anonymous=True) 
            object_detector = object_subscriber_node.ObjectDetectionNode()
            gs = sm_grid_search
            gs_points = gs.GridSearch(5, 5, 3, userdata.rem_loc[0][0], userdata.rem_loc[0][1])  # define multiple target points here: cartesian
            targets = gs.square_target()
            gs_traversal_object = sm_grid_search.GS_Traversal(0.6, 0.3, targets)
            mallet_detected = rospy.Subscriber("waterbottle_detected", Bool, queue_size = 1)
            obj1_in_correct_loc = gs_traversal_object.navigate("OBJ2") #publishing messages?

            if mallet_detected:
                self.glob_msg.pub_state("Grid Search did find Waterbottle")
                if obj1_in_correct_loc:
                    self.glob_msg.pub_state("Close enough to Waterbottle") 
                    signal_green_led()

                else:
                    self.glob_msg.pub_state("Grid Search did not find Waterbottle")
        else:
            self.glob_msg.pub_state("Did not reach Waterbottle GNSS")
        userdata.prev_loc = userdata.rem_loc[0]
        userdata.rem_loc.remove(self.__class__.__name__) #remove state from location list
        return "Location Selection"
    
    
class ABORT(smach.State):  # Abort State --> In case of failure what should it do? When should it be called?
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["Abort Complete", "Abort Failed"],
            input_keys=["prev_loc", "start_location"],
        )
        self.glob_msg = None

    def set_msg(self, glob_msg: GLOB_MSGS):
        self.glob_msg = glob_msg

    def execute(self, userdata):
        # Determine the location to return to
        if not userdata.prev_loc:
            self.glob_msg.pub_state("No previous location available; returning to start location.")
            return_location = userdata.start_location
        elif len(userdata.prev_loc) == 1 and userdata.prev_loc[0] == "start":
            self.glob_msg.pub_state("Aborting back to start location as it's the first task.")
            return_location = userdata.start_location
        else:
            return_location = userdata.prev_loc

        self.glob_msg.pub_state(f"Aborting to location: {return_location}")

        try:
            # Perform straight-line traversal back to the determined location
            sla = StraightLineApproach(0.6, 0.3, return_location)
            sla.navigate()

            # Check final distance to the target location
            current_pose = self.glob_msg.get_pose()
            distance = math.sqrt(
                (current_pose.pose.position.x - return_location[0]) ** 2
                + (current_pose.pose.position.y - return_location[1]) ** 2
            )

            if distance < 2.0:  # Threshold for successful navigation
                self.glob_msg.pub_state("Successfully returned to the abort location.")
                signal_green_led()
                return "Location Selection"
            else:
                self.glob_msg.pub_state("Failed to reach the abort location within threshold.")
                return "Location Selection"

        except rospy.ROSInterruptException as e:
            self.glob_msg.pub_state(f"ROS Interrupt Exception during abort: {e}")
            return "Location Selection"


class TasksEnded(smach.State):
    def __init__(self):
        smach.State.__init__(self)
    
    def execute(self, userdata):
        print("End of tasks")


def main():
    rospy.init_node('RSX_Rover')
    #gui_status = rospy.Publisher('gui_status', String, queue_size=10) #where should be used?
    
    sm = smach.StateMachine(outcomes=["Tasks Ended"])
    sm.userdata.cartesian = {}

    glob_msg = GLOB_MSGS()
    init = InitializeAutonomousNavigation()
    gnss1 = GNSS1()
    gnss2 = GNSS2()
    ar1 = AR1()
    ar2 = AR2()
    ar3 = AR3()
    obj1 = OBJ1()
    obj2 = OBJ2()
    #abort = ABORT() 

    init.set_msg(glob_msg)
    gnss1.set_msg(glob_msg)
    gnss2.set_msg(glob_msg)
    ar1.set_msg(glob_msg)
    ar2.set_msg(glob_msg)
    ar3.set_msg(glob_msg)
    obj1.set_msg(glob_msg)
    obj2.set_msg(glob_msg)
    #abort.set_msg(glob_msg)

    with sm:
        smach.StateMachine.add(
            "Initialize Autonomous Navigation",
            init,
            transitions={"Tasks Execute": "Tasks Execute"},
            remapping={"cartesian": "cartesian"}
        )
        
        # Define the tasks execution sub-state machine
        sm_tasks_execute = smach.StateMachine(outcomes=["Tasks Ended"])
        sm_tasks_execute.userdata.locations_list_c = sm.userdata.cartesian.copy()
        #sm_tasks_execute.userdata.start_location = sm_tasks_execute.userdata.locations_list_c.pop("start")
        sm_tasks_execute.userdata.rem_loc = sm_tasks_execute.userdata.locations_list_c.copy()
        sm_tasks_execute.userdata.prev_loc = None  # Initialize with no previous location

        with sm_tasks_execute:
            smach.StateMachine.add(
                "Location Selection",
                LocationSelection(),
                transitions={
                    "GNSS1": "GNSS1",
                    "GNSS2": "GNSS2",
                    "AR1": "AR1",
                    "AR2": "AR2",
                    "AR3": "AR3",
                    "OBJ1": "OBJ1",
                    "OBJ2": "OBJ2",
                },
                remapping={
                    "rem_loc": "rem_loc",
                    "locations_list_c": "locations_list_c",
                    "prev_loc": "prev_loc",  # Add remapping for prev_loc
                    "start_location" : "start_location"
                }
            )
            
            smach.StateMachine.add(
                "GNSS1",
                gnss1,
                transitions={"Location Selection": "Location Selection"},
                remapping={
                    "rem_loc": "rem_loc",
                    "prev_loc": "prev_loc"  # Output the last completed location
     
                }
            )
            
            smach.StateMachine.add(
                "GNSS2",
                gnss2,
                transitions={"Location Selection": "Location Selection"},
                remapping={
                    "rem_loc": "rem_loc",
                    "prev_loc": "prev_loc"
                    
                }
            )
            
            smach.StateMachine.add(
                "AR1",
                ar1,
                transitions={"Location Selection": "Location Selection"},
                remapping={
                    "rem_loc": "rem_loc",
                    "prev_loc": "prev_loc"

                }
            )
            
            smach.StateMachine.add(
                "AR2",
                ar2,
                transitions={"Location Selection": "Location Selection"},
                remapping={
                    "rem_loc": "rem_loc",
                    "prev_loc": "prev_loc"
                    
                }
            )
            
            smach.StateMachine.add(
                "AR3",
                ar3,
                transitions={"Location Selection": "Location Selection"},
                remapping={
                    "rem_loc": "rem_loc",
                    "prev_loc": "prev_loc"

                }
            )
            
            smach.StateMachine.add(
                "OBJ1",
                obj1,
                transitions={"Location Selection": "Location Selection"},
                remapping={
                    "rem_loc": "rem_loc",
                    "prev_loc": "prev_loc"
                    
                }
            )
            
            smach.StateMachine.add(
                "OBJ2",
                obj2,
                transitions={"Location Selection": "Location Selection"},
                remapping={
                    "rem_loc": "rem_loc",
                    "prev_loc": "prev_loc"
                }
            )
            
            #smach.StateMachine.add(
            #    "Abort",
            #    abort,
            #    transitions={
            #        "Abort Complete": "Location Selection",
            #        "Abort Failed": "Tasks Ended", #What should do in this case?
            #    },
            #    remapping= {
            #        "prev_loc": "prev_loc",  # Use the previous location for abort
            #        "start_location" : "start_location"
            #    }
            #)
            
        smach.StateMachine.add(
            "Tasks Execute",
            sm_tasks_execute,
            transitions={"Tasks Ended": "Tasks Ended"}
        )
        
        smach.StateMachine.add(
            "Task Ended",
            TasksEnded()
        )
        

    outcome = sm.execute()


#We don't need to wait on the GUI we can tell it to initialize
#Press initialize on GUI -- asks for the task locations and the current location automatically, then performs initialization
#In case of swithcing to manual control what should be done?
#Abort mode -- go back to the previous location, if its the first task it should go to the start location +
#When/How should the abort state be called? What should it do in case of failure?
#And if a task fails go to the previous location, try the next one, and put the other one at the end
#There should be a case that also considers going to the previous task teleop


if __name__ == "__main__":
    print("Starting the autonomous missions")
    main()