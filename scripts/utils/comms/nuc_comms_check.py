#!/usr/bin/env python3

import rospy
import subprocess
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool 

def main():
    pub_network = rospy.Publisher('network_status', Bool, queue_size=1)
    rospy.init_node("nuc_comms_check")
    host = rospy.get_param("base_ip", "192.168.0.123") # 
    net_stat = Bool()

    # we continuously send pings to check network communication is working
    while not rospy.is_shutdown():
        """ 
        For linux:
        -c followed by a number is the number of pings to be sent
        -w followed by a number is how many seconds to wait for a response
        """
        command = ['ping', '-c', '2', '-w', '2', host]
        connected = subprocess.run(command, capture_output=True)
        if connected.returncode == 0:
            net_stat = True
        else:
            net_stat = False

        pub_network.publish(net_stat)
        time.sleep(0.001) # this is the amount of time (in seconds) waiting before checking the connection again

if __name__ == '__main__':
    main()
