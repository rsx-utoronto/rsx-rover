#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Odometry

class GPSTesting: 

    def __init__(self):

        self.gps_sub = rospy.Subscriber("/gps_node/fix", NavSatFix, self.gps_callback)
        self.utm_pub = rospy.Publisher("/utm", NavSatFix)

    
    def gps_callback(self, msg):

        # read in gps 
        self.gps_msg = msg

        
        # run the utm conversion 
        utm_msg = Odometry()
        
        # republish as utm
        self.utm_pub(utm_msg)