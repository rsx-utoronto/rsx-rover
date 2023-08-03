#!/usr/bin/env python

import rospy
import subprocess
import time
from std_msgs.msg import Bool

def main():
    pub_network = rospy.Publisher('network_status', Bool, queue_size=1)
    rospy.init_node("base_comms_check")
    host = rospy.get_param("nuc_ip", "192.168.0.99")
    net_stat = Bool()

    # we continuously send pings to check network communication is working
    while not rospy.is_shutdown():
        """ 
        For linux:
        -c followed by a number is the number of pings to be sent
        -w followed by a number is how many seconds to wait for a response
        """
        # if running this script in a container, make sure ping is installed
        command = ['ping', '-c', '2', '-w', '2', host]
        connected = subprocess.run(command, capture_output=True)
        if connected.returncode == 0:
            net_stat = True 
        else:
            net_stat = False
        print(net_stat)
        pub_network.publish(net_stat)
        time.sleep(0.001)

if __name__ == '__main__':
    main()