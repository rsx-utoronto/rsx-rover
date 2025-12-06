#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import numpy as np


class SpeedCalibration(rclpy.node.Node):

    def __init__(self):
        super().__init__('calibration_test')
        self.pub = self.create_publisher(Twist, '/drive', 1)
        self.sub = self.create_subscription(Joy, '/joy', self.callback, 10)
        self.rate = self.create_rate(100)
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
    rclpy.init()
    speed_test = SpeedCalibration()
    start = rclpy.time.Time.now()
    curr = rclpy.time.Time.now()
    while rclpy.ok():
        while (curr - start) < rclpy.duration.Duration(seconds=10):
            curr = rclpy.time.Time.now()
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
    rclpy.spin(speed_test)

if __name__ == "__main__":
    main()
