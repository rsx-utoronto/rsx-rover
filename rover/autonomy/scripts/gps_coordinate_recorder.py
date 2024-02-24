#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix

class GPSRecorder:
    def __init__(self):
        self.gps_sub = rospy.Subscriber('/ublox/fix', NavSatFix, self.callback)
        self.gps_data_file = open('gps_data.txt', 'w')
        self.r = rospy.Rate(0.1)

    def callback(self, msg):
        self.gps_data_file.write(str(msg.latitude) + ', ' + str(msg.longitude) + '\n')
        self.r.sleep()

if __name__ == '__main__':
    rospy.init_node('gps_recorder')
    gps_recorder = GPSRecorder()
    rospy.spin()
    gps_recorder.gps_data_file.close()
    