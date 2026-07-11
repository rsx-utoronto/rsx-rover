#!/usr/bin/python3

import rospy
from sensor_msgs.msg import NavSatFix

def get_new_target():
    pub = rospy.Publisher('gps_target', NavSatFix, queue_size=10)
    rospy.init_node('gps_target_waypoint_updater')
    while not rospy.is_shutdown():
        target = NavSatFix()
        target.latitude = float(input("enter latitude: "))
        target.longitude = float(input("enter longitude: "))
        pub.publish(target)
        print("published new target")
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        get_new_target()
    except rospy.ROSInterruptException:
        pass