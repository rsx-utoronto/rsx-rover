#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def fake_status_publisher():
    rospy.init_node('fake_status_publisher', anonymous=True)
    pub = rospy.Publisher('gui_status', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    i = 0
    while not rospy.is_shutdown():
        status_message = "Fake status message" + str(rospy.get_time())
        rospy.loginfo(status_message)
        rate.sleep()
        i += 1
        if i % 10 == 0:
            pub.publish("Goal Point Reached: GNSS1")
        else:
            pub.publish(status_message)
            
if __name__ == '__main__':
    try:
        fake_status_publisher()
    except rospy.ROSInterruptException:
        pass