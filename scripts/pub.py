#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Int64 

def main():
    rospy.init_node("publish")
    pub = rospy.Publisher('publish', Int64, queue_size=1)
    pub2 = rospy.Publisher('publish2', Int64, queue_size=1)
    
    count = Int64()
    count2 = Int64()
    # we continuously send pings to check network communication is working
    counter = 1
    while not rospy.is_shutdown():
        pub.publish(count)
        pub2.publish(count2)
        print(count)
        counter += 1
        count.data = counter
        count2.data = counter + 100000
        time.sleep(0.01) # this is the amount of time (in seconds) waiting before checking the connection again

if __name__ == '__main__':
    main()