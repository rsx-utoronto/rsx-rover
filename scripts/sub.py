#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Bool, Float64

def callback(data):
    rospy.loginfo(rospy.get_caller_id() +  'I heard %s', data.data)

def main():
    rospy.init_node('subscribe', anonymous=True)

    while not rospy.is_shutdown():
        rospy.Subscriber('gui', Bool, callback, queue_size=1)
        rospy.Subscriber('slider', Float64, callback, queue_size=1)
    
        time.sleep(0.1)

if __name__ == '__main__':
    main()