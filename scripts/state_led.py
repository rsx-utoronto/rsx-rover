#!/usr/bin/env python3

import rospy
from rover.msg import StateMsg
import serial


def state_callback(msg):
  global res
  mode = msg.rover_mode 
  if mode == 'IDLE' or mode == 'MANUAL':
    res = 'm'
  
  serial_port = "/dev/ttyUSB0"
  baud_rate = 9600
  ser = serial.Serial(serial_port, baud_rate, timeout=1)
  ser.open()
  ser.write(res.encode())
  ser.close()



rospy.init_node('state_listener', anonymous=True)
rospy.Subscriber("/rover_state", StateMsg, state_callback)


# spin() simply keeps python from exiting until this node is stopped
rospy.spin()

