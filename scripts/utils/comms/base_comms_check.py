#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import subprocess
import time
from std_msgs.msg import Bool

def main():
    rclpy.init()
    node=rclpy.create_node('base_comms_check')
    pub_network = node.create_publisher( Bool, 'network_status',1)
    
    # host = rospy.get_param("nuc_ip", "192.168.0.99")
    
    node.declare_parameter('nuc_ip', '192.168.0.99')
    host = node.get_parameter('nuc_ip').get_parameter_value().string_value
    
    net_stat = Bool()

    # we continuously send pings to check network communication is working
    while rclpy.ok():
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
        # pub_network.publish(net_stat)
        pub_network.publish(Bool(data=net_stat))
        time.sleep(0.001)

if __name__ == '__main__':
    main()