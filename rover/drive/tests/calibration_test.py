#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import numpy as np


class SpeedCalibration:

    def __init__(self):
        self.pub = rospy.Publisher('/drive', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/joy', Joy, self.callback)
        self.rate = rospy.Rate(100)
        self.joy_msg = Joy()
        self.kill = False
        print(self.joy_msg)

    def callback(self, msg):
        self.joy_msg = msg    
        if self.joy_msg.buttons[10] == 1 and self.kill == False: 
            self.kill = True
        elif self.joy_msg.buttons[10] == 1 and self.kill == True:
            self.kill = False

def main():
    rospy.init_node('calibration_test')
    speed_test = SpeedCalibration()
    start = rospy.Time.now()
    curr = rospy.Time.now()
    while not rospy.is_shutdown():
        while (curr - start) < rospy.Duration(10):
            curr = rospy.Time.now()
            move = Twist()
            x = 0
            z = np.pi/20

            if speed_test.kill == False:
                move.linear.x = 0.1*x
                move.angular.z = z
            else:
                move.linear.x = 0.0
                move.angular.z = 0.0

            speed_test.pub.publish(move)
            speed_test.rate.sleep()
        move.linear.x = 0.0
        move.angular.z = 0.0
        speed_test.pub.publish(move)
    speed_test.rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    main()
