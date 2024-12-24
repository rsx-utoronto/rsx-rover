#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

def init_map():

    rospy.init_node('init_map')
    utm = rospy.wait_for_message("/utm", Odometry)

    map_origin_point = PointStamped()
    map_origin = Marker()
    map_origin.header.stamp = rospy.Time.now()
    map_origin.header.frame_id = "map"

    map_origin.pose.position.x = utm.pose.pose.position.x
    map_origin.pose.position.y = utm.pose.pose.position.y
    map_origin.pose.position.z = utm.pose.pose.position.z


    map_origin.id = 0
    map_origin.type = Marker.TEXT_VIEW_FACING
    map_origin.scale.z = 2  # Scale z specifies the height of an uppercase "A"
    map_origin.color.a = 1.0  # Alpha
    map_origin.color.r = 1.0  # Red

    map_origin.text = "STARTING POINT"

    map_origin_point.header.stamp = rospy.Time.now()
    map_origin_point.header.frame_id = "map"

    map_origin_point.point.x = utm.pose.pose.position.x
    map_origin_point.point.y = utm.pose.pose.position.y - 2
    map_origin_point.point.z = utm.pose.pose.position.z



    map_point_pub = rospy.Publisher("map_origin_point", PointStamped, queue_size=1, latch=True)
    map_pub = rospy.Publisher("map_origin", Marker, queue_size=1, latch=True)
    map_pub.publish(map_origin)
    map_point_pub.publish(map_origin_point)


    rospy.spin()

if __name__ == '__main__':
    init_map()