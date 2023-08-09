#!/usr/bin/env python3

import serial
import rospy
from std_msgs.msg import String

ser = serial.Serial('/dev/ttyUSB0', 115200)
rospy.init_node('receive_signal')
pub = rospy.Publisher('/arduino_signal', String, queue_size=10)
rate = rospy.Rate(1) # 1hz

while not rospy.is_shutdown():
    read_serial = ser.readline().decode('utf-8').strip()
    print(read_serial)
    pub.publish(read_serial)
    rate.sleep()


