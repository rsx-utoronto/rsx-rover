#!/usr/bin/env python3
import rostopic
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String, Float32MultiArray
import time
from rover.msg import UnityCmd

class TranslatorNode():
    def __init__(self):
        rate = rospy.Rate(2)
        self.topics = ["drive"]

        
        self.arm_angles = []
        self.vel = Twist()
        self.start_time = 0

        self.arm_sub = rospy.Subscriber("arm_cmd", Float32MultiArray, self.arm_Callback())
        self.vel_sub = rospy.Subscriber("rover_vel", Twist, self.vel_Callback())
        self.unity_publisher = rospy.Publisher("cmd", UnityCmd, queue_size= 10)

        while not rospy.is_shutdown():
            self.update()

    def arm_Callback(self, input):
        self.arm_angles = input.data
    def vel_Callback(self, input):
        self.vel = input
    

    def update(self):
        msg = UnityCmd()
        msg.data = self.arm_angles
        msg.linear = self.vel.linear
        msg.angular = self.vel.angular
        self.unity_publisher.publish(msg)

    def main(self):
        self.test.publish("bluh")


if __name__ == '__main__':
    rospy.init_node('Translator')
    node = TranslatorNode()
    rospy.spin()
