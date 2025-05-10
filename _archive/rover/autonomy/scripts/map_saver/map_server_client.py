#! /usr/bin/env python

import rospy
from nav_msgs.srv import GetMap, GetMapRequest

rospy.init_node("map_server_client")
rospy.wait_for_service("/nav_msgs/GetMap")

rate = rospy.Rate(1)


service_client = rospy.ServiceProxy("/nav_msgs/GetMap", GetMap)
service_object = GetMapRequest()

while not rospy.is_shutdown():
    result = service_client(service_object)
    rospy.loginfo("New Map loaded")
    rate.sleep()

