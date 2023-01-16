import rospy
import subprocess
import time
from std_msgs.msg import String

def main(host):
    # we continuously send pings to check network communication is working
    while not rospy.is_shutdown():
        """ 
        For linux:
        -c followed by a number is the number of pings to be sent
        -w followed by a number is how many milliseconds to wait for a response
        """
        command = "ping -c 2 -w 1 %s" % (host) 
        connected = subprocess.call(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        if connected == 0:
            rospy.loginfo('Communication is working.')
        else:
            rospy.loginfo('Communication is NOT working.')  
        time.sleep(2)

if __name__ == '__main__':
    host = 0 # this should be the ip that communication is checked for (i.e., will be pinged)
    main(host)