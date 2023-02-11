#!/usr/bin/env python

import rospy
import subprocess
import time

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
        # if running this script in a container, make sure ping is installed
        command = ['ping', '-c', '2', '-w', '2', host]
        connected = subprocess.run(command, capture_output=True)
        if connected == 0:
            rospy.loginfo('Connected')
        else:
            rospy.loginfo('NOT Connected')
        time.sleep(2)

if __name__ == '__main__':
    main()
