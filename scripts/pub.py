#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Int64 

def main():
    pub = rospy.Publisher('publish', Int64, queue_size=1)
    rospy.init_node("publish")
    count = Int64()
    # we continuously send pings to check network communication is working
    counter = 1
    while not rospy.is_shutdown():
        pub.publish(count)
        print(count)
        counter += 1
        count.data = counter
        time.sleep(0.01) # this is the amount of time (in seconds) waiting before checking the connection again

if __name__ == '__main__':
    main()