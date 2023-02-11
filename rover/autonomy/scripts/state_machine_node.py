import rospy
from rsx_rover.msg import GPSMsg, StateMsg
from std_msgs.msg import String

goal_gps_list = [] #populate

gps_index = 0

ar_tag_goal_id = 0

light = 'red'

GPS_found = False
AR_found = False
searching = False
MISSION_OVER = False

def publish_state():

    global goal_gps_list, gps_index, ar_tag_goal_id, light, GPS_found, AR_found, searching,  MISSION_OVER

    pub = rospy.Publisher('state_machine_node_pub', StateMsg, queue_size=10)

    gps = GPSMsg()
    gps.latitude = goal_gps_list[gps_index][0]
    gps.longitude = goal_gps_list[gps_index][1]
    msg = StateMsg()
    msg.next_goal_gps = gps
    msg.next_ar_tag_id = ar_tag_goal_id
    msg.light = light
    msg.GPS_found = GPS_found
    msg.AR_found = AR_found
    msg.searching = searching
    msg.MISSION_OVER = MISSION_OVER

    if not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
    
def gps_find_callback(data):
    
    global searching

    if bool(data.data) and not AR_found:
        searching = True
    publish_state()

def ar_find_callback(data):

    global AR_found, searching, light
    
    if bool(data.data):
        AR_found = True
        searching = False
        light = 'green'
    
    publish_state()

def done_checkpoint_callback(data):

    global gps_index, ar_tag_goal_id, MISSION_OVER, light, GPS_found, AR_found

    if bool(data.data):
        if gps_index == 4 and ar_tag_goal_id == 4:
            MISSION_OVER = True
            light = 'blue'
        else:
            ar_tag_goal_id += 1
            gps_index += 1
            GPS_found, AR_found = False
            light = 'red'
    
    publish_state()

def listener():
    rospy.init_node('state_machine_node', anonynous=True)
    rospy.Subscriber('gps_find', gps_find_callback) #change node name
    rospy.Subscriber('ar_find', ar_find_callback) #change node name
    rospy.Subscriber('done_checkpoint', done_checkpoint_callback) #change node name

    rospy.spin()

if __name__ == '__main__':
    listener()


