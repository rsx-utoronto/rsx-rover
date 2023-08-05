import rospy
from rover.msg import StateMsg
from rover.srv import AddGoal
from std_msgs.msg import String





class StateMachineNode:

    def __init__(self):

        self.state_msg = StateMsg()


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

    
    

    
#     def state_listener(self):

#         """
#         subscribes to gps node if its not set to none
#         otherwise subscribes to the first task in the list
#         """

#         rospy.init_node('state_machine_node', anonymous=True)

#         if self.gps_node is not None:
#             rospy.Subscriber(self.gps_node, self.gps_find_callback)
#         else:
#             rospy.Subscriber(
#                 self.node_dict[self.task_id_lst[0]], self.task_callback)



