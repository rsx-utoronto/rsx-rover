#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from add_goal import GPS_to_UTM




def main():
    rospy.init_node('map_debris')

    debris = [
        (380739.16, 5701615.04),
        (380756.93, 5701628.64),
        (380746.93, 5701652.69),
        (380728.74, 5701668.80)
    ]

    debris_1_pub = rospy.Publisher("map_debris_1", PointStamped, queue_size=1, latch=True)
    debris_2_pub = rospy.Publisher("map_debris_2", PointStamped, queue_size=1, latch=True)
    debris_3_pub = rospy.Publisher("map_debris_3", PointStamped, queue_size=1, latch=True)
    debris_4_pub = rospy.Publisher("map_debris_4", PointStamped, queue_size=1, latch=True)


    debris_1 = PointStamped()
    debris_2 = PointStamped()
    debris_3 = PointStamped()
    debris_4 = PointStamped()

    map_origin = rospy.wait_for_message("/map_origin_point", PointStamped)

    debris_1.header.stamp = rospy.Time.now()
    debris_1.header.frame_id = "map"
    debris_1.point.x = debris[0][0] - map_origin.point.x
    debris_1.point.y = debris[0][1] - map_origin.point.y
    debris_1.point.z = map_origin.point.z

    debris_2.header.stamp = rospy.Time.now()
    debris_2.header.frame_id = "map"
    debris_2.point.x = debris[1][0] - map_origin.point.x
    debris_2.point.y = debris[1][1] - map_origin.point.y
    debris_2.point.z = map_origin.point.z

    debris_3.header.stamp = rospy.Time.now()
    debris_3.header.frame_id = "map"
    debris_3.point.x = debris[2][0] - map_origin.point.x
    debris_3.point.y = debris[2][1] - map_origin.point.y
    debris_3.point.z = map_origin.point.z

    debris_4.header.stamp = rospy.Time.now()
    debris_4.header.frame_id = "map"
    debris_4.point.x = debris[3][0] - map_origin.point.x
    debris_4.point.y = debris[3][1] - map_origin.point.y
    debris_4.point.z = map_origin.point.z

    debris_1_pub.publish(debris_1)
    debris_2_pub.publish(debris_2)
    debris_3_pub.publish(debris_3)
    debris_4_pub.publish(debris_4)


    rospy.spin()
if __name__ == "__main__":
    main()