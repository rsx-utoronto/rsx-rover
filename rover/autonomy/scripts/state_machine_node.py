import rospy
from rsx_rover.msg import GPSMsg, StateMsg
from std_msgs.msg import String

class StateMachineNode:

    def __init__(self, task_id_lst, task_dict, node_dict, gps_node=None, gps_loc_lst=[]):

        '''
        gps_loc_lst is optional and type list[GPSMsg] - should be the list of GPS coordinates to visit in order
        task_id_lst is type list[int] - list of the ids of tasks to trigger in order
        task_dict is type dict[int, str] - a dictionary mapping task id to a description
        node_dict is type dict[int, str] - a dictionary mapping task id to the node associated with it
        gps_node is optional and type str - the node which publishes if the intended gps coordinate is reached
        '''

        self.gps_loc_lst = gps_loc_lst
        self.task_id_lst = task_id_lst
        self.task_dict = task_dict
        self.node_dict = node_dict
        self.gps_node = gps_node

        self.index = 0
        self.next_goal_gps = None if self.gps_loc_lst == [] else self.gps_loc_lst[0]
        self.goal_gps_found = False
        self.current_task = self.task_dict[self.task_id_lst[0]]
        self.MISSION_OVER = False


    def publish_state(self):


        pub = rospy.Publisher('state_machine_node_pub', StateMsg, queue_size=10)
        msg = StateMsg()
        msg.next_goal_gps = self.next_goal_gps
        msg.goal_gps_found = self.goal_gps_found
        msg.current_task = self.current_task
        msg.MISSION_OVER = self.MISSION_OVER

        if not rospy.is_shutdown():
            rospy.loginfo(msg)
            pub.publish(msg)

    
    def gps_find_callback(self, data):

        """
        After the goal gps location is reached, the node for the associated task is subscribed to
        """

        if bool(data.data):
            self.goal_gps_found = True
            node_name = self.node_dict[self.task_id_lst[self.index]]
            rospy.Subscriber(node_name, self.task_callback)
        
        self.publish_state()


    def task_callback(self, data):

        """
        if this is the last task in the list, then its mission over
        otherwise, the list index is incremented, the goal gps nd current task are updated 
        """

        if bool(data.data):
            self.index += 1

            if self.index == len(self.task_id_lst):
                self.MISSION_OVER = True
            else:

                if self.gps_loc_lst != []:
                    self.next_goal_gps = self.gps_loc_lst[self.index]
                    self.goal_gps_found = False
                self.current_task = self.task_dict[self.task_id_lst[self.index]]

        self.publish_state()    

    
    def listener(self):

        """
        subscribes to gps node if its not set to none
        otherwise subscribes to the first task in the list
        """

        rospy.init_node('state_machine_node', anonymous=True)

        if self.gps_node is not None:
            rospy.Subscriber(self.gps_node, self.gps_find_callback)
        else:
            rospy.Subscriber(
                self.node_dict[self.task_id_lst[0]], self.task_callback)



