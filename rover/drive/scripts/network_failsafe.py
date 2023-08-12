#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class NetworkFailsafe:
    def __init__(self):
        self.status_sub = rospy.Subscriber("/network_status", Bool, self.status_callback)
        self.drive_pub = rospy.Publisher("/drive", Twist, queue_size=1)
        self.rate = rospy.Rate(10)
        self.twist = Twist()
    def status_callback(self, msg):
        if msg.data == False:
            rospy.loginfo("KILL ENGAGED")
            # self.twist = Twist()
            self.twist.linear.x = 0
            self.twist.linear.y = 0
            self.twist.linear.z = 0
            self.twist.angular.x = 0
            self.twist.angular.y = 0
            self.twist.angular.z = 0
            # self.drive_pub.publish(twist)
            # self.rate.sleep()
        else:
            pass

def main():
    rospy.init_node("network_failsafe")
    nf = NetworkFailsafe()
    while not rospy.is_shutdown():
        nf.drive_pub.publish(nf.twist)
        nf.rate.sleep()
        
    rospy.spin()

if __name__ == '__main__':
    main()

