import rospy
from rsx_rover.msg import GPSMsg, StateMsg
from std_msgs.msg import String

class StateMachineNode:
    goal_gps_list = [] #populate

    gps_index = 0

    ar_tag_goal_id = 0 #id for 4x4_50 library for AR tag

    light = 'red'

    GPS_found = False
    AR_found = False
    searching = False
    MISSION_OVER = False


    def publish_state(self):

      
        pub = rospy.Publisher('state_machine_node_pub', StateMsg, queue_size=10)

        gps = GPSMsg()
        gps.latitude = self.goal_gps_list[self.gps_index][0]
        gps.longitude = self.goal_gps_list[self.gps_index][1]
        msg = StateMsg()
        msg.next_goal_gps = gps
        msg.next_ar_tag_id = self.ar_tag_goal_id
        msg.light = self.light
        msg.GPS_found = self.GPS_found
        msg.AR_found = self.AR_found
        msg.searching = self.searching
        msg.MISSION_OVER = self.MISSION_OVER

        if not rospy.is_shutdown():
            rospy.loginfo(msg)
            pub.publish(msg)
    
    def gps_find_callback(self, data):

        if bool(data.data) and not self.AR_found:
            self.searching = True
        self.publish_state()

    def ar_find_callback(self, data):
        
        if bool(data.data):
            self.AR_found = True
            self.searching = False
            self.light = 'green'
        
        self.publish_state()

    def done_checkpoint_callback(self, data):

        if bool(data.data):
            if self.gps_index == 4 and self.ar_tag_goal_id == 4:
                self.MISSION_OVER = True
                self.light = 'blue'
            else:
                self.ar_tag_goal_id += 1
                self.gps_index += 1
                self.GPS_found, AR_found = False
                self.light = 'red'
        
        self.publish_state()

    def listener(self):
        rospy.init_node('state_machine_node', anonymous=True)
        rospy.Subscriber('gps_find', self.gps_find_callback) #change node name
        rospy.Subscriber('ar_find', self.ar_find_callback) #change node name
        rospy.Subscriber('done_checkpoint', self.done_checkpoint_callback) #change node name

        rospy.spin()

if __name__ == '__main__':
    state_machine_node = StateMachineNode()
    state_machine_node.listener()


