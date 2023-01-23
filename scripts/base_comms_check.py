#!/usr/bin/env python

import rospy
import subprocess
import time
from std_msgs.msg import String

def main():
    rospy.init_node("base_comms_check")
    host = rospy.get_param("jetson_ip", "192.168.0.250")
    # we continuously send pings to check network communication is working
    while not rospy.is_shutdown():
        """ 
        For linux:
        -c followed by a number is the number of pings to be sent
        -w followed by a number is how many milliseconds to wait for a response
        """
        command = "ping -c 2 -w 500 %s" % (host) 
        connected = subprocess.call(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        if connected == 0:
            rospy.loginfo('connected')
        else:
            rospy.loginfo('NOT connected')
        time.sleep(2) # this is the amount of time (in seconds) waiting before checking the connection again

if __name__ == '__main__':
    main()