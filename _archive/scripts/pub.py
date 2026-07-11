#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Int64 
from geometry_msgs.msg import Twist, PoseStamped
from rsx_rover.msg import GPSMsg, StateMsg

def main():
    rospy.init_node("publish")
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    pub2 = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    
    goal = PoseStamped()
    goal.pose.position.x = 1.0
    goal.pose.position.y = 2.0
    goal.pose.position.z = 0.0
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0
    twist = Twist()
    twist.linear.x = 2
    twist.linear.y = 0
    twist.angular.x = 4
    twist.angular.z = 0
    # we continuously send pings to check network communication is working
    counter = 1
    while not rospy.is_shutdown():
        pub.publish(goal)
        pub2.publish(twist)
        print(counter)
        counter += 1
        time.sleep(0.01) # this is the amount of time (in seconds) waiting before checking the connection again

if __name__ == '__main__':
    main()