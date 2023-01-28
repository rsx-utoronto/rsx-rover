#!/usr/bin/env python

import rospy
import subprocess
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist 

<<<<<<< HEAD:scripts/comms_check.py
def main(host):

    pub = rospy.Publisher('drive',Twist, queue_size=10)
=======
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
<<<<<<< HEAD:scripts/comms_check.py
            rospy.loginfo('Communication is working.')
            print('connected')
        else:
            rospy.loginfo('Communication is NOT working.') 
            print('not connected') 
            cmd_vel = Twist()
            cmd_vel.linear.x = 0
            cmd_vel.linear.y = 0
            cmd_vel.linear.z = 0
            cmd_vel.angular.z = 0

            pub.publish(cmd_vel)
            
        time.sleep(2)

if __name__ == '__main__':
    host = '192.168.0.69' # this should be the ip that communication is checked for (i.e., will be pinged)
    main(host)
=======
            rospy.loginfo('connected')
        else:
            rospy.loginfo('NOT connected')
        time.sleep(2) # this is the amount of time (in seconds) waiting before checking the connection again

if __name__ == '__main__':
    main()
>>>>>>> b2a7409a646fc95ab7932705a1150f352ee1a2d5:scripts/base_comms_check.py
