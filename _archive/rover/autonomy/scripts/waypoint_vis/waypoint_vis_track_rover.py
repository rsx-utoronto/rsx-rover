#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

def main():

    rospy.init_node('track_rover')
    rover_pub = rospy.Publisher("map_rover_pos", PointStamped, queue_size=1, latch=True)
    
    while(True):

        rover_pub.publish(utm_cb())
        rospy.sleep(0.1)


def utm_cb():
    rover_pos = PointStamped()

    utm = rospy.wait_for_message("/utm", Odometry)
    map_origin = rospy.wait_for_message("/map_origin_point", PointStamped)

    rover_pos.point.x = utm.pose.pose.position.x - map_origin.point.x
    rover_pos.point.y = utm.pose.pose.position.y - map_origin.point.y
    rover_pos.point.z = utm.pose.pose.position.z - map_origin.point.z


    rover_pos.header.stamp = rospy.Time.now()
    rover_pos.header.frame_id = "map"

    return rover_pos

if __name__ == '__main__':
    main()