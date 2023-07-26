#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class SpeedCalibration:

    def __init__(self):
        self.pub = rospy.Publisher('/drive', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/joy', Joy, self.callback)
        self.rate = rospy.Rate(1)
        self.joy_msg = Joy()
        print(self.joy_msg)

    def callback(self, msg):
        print(msg)
        self.joy_msg = msg    

def main():
    rospy.init_node('calibration_test')
    speed_test = SpeedCalibration()
    while not rospy.is_shutdown():
        time = 10
        i = 0
        while i < time:
            move = Twist()
            x = 2
            z = 0
            move.linear.x = 0.1*x
            move.angular.z = 0.1*z
            speed_test.pub.publish(move)
            i+=1
            try:
                if speed_test.joy_msg.buttons[10] == 1:
                    move.linear.x = 0.0
                    move.angular.z = 0.0
                    speed_test.pub.publish(move)
                    break 
            except: 
                continue
            speed_test.rate.sleep()

if __name__ == "__main__":
    main()
