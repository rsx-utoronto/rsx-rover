#!/usr/bin/env python

import rospy
import subprocess
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def main():
    pub = rospy.Publisher('drive', Twist, queue_size=10)
    rospy.init_node("jetson_comms_check")
    host = rospy.get_param("base_ip", "192.168.0.69")
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
            # placeholder for allowing driving
            pass
        else:
            msg = Twist()
            pub.publish(msg)
        time.sleep(2) # this is the amount of time (in seconds) waiting before checking the connection again

if __name__ == '__main__':
    main()