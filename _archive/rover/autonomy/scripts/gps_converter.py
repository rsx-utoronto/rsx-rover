#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from inertial_sense_ros.msg import GPS


class GPSConverter: 
        
        def __init__(self):
                
            self.gps_sub = rospy.Subscriber("gps", GPS, self.gps_callback)
            self.navsatfix_pub = rospy.Publisher("NavSatFix", NavSatFix, queue_size=1)
            self.navsatfix_msg = NavSatFix()

        def gps_callback(self,msg):

            self.navsatfix_msg.header.stamp = msg.header.stamp
            self.navsatfix_msg.header.frame_id = msg.header.frame_id
            self.navsatfix_msg.status.status = 1

            self.navsatfix_msg.latitude = msg.latitude
            self.navsatfix_msg.longitude = msg.longitude
            self.navsatfix_msg.altitude = msg.altitude

            varH = (msg.hAcc/1000.0)**2
            varV = (msg.vAcc/1000.0)**2

            self.navsatfix_msg.position_covariance[0] = varH
            self.navsatfix_msg.position_covariance[4] = varH
            self.navsatfix_msg.position_covariance[8] = varV

            self.navsatfix_msg.position_covariance_type = 2


def main():
    rospy.init_node("gps_converter")

    gps_converter = GPSConverter()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        gps_converter.navsatfix_pub.publish(gps_converter.navsatfix_msg)
        rate.sleep()
    rospy.spin()


if __name__ == "__main__":
    main()