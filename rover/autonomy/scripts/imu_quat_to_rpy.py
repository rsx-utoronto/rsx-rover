#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import Imu
import tf
import rosbag
from geometry_msgs.msg import PoseStamped

class IMUQuatToRPY:
    def __init__(self):
        self.imu_sub = rospy.Subscriber('imu/orient', Imu, self.imu_callback)
        # self.imu_pub = rospy.Publisher('/imu_2d', Imu, queue_size=10)
        self.prev_time = rospy.Time.now()
        self.imu_2d = Imu()

    def imu_callback(self, msg):
        if msg.header.stamp <= self.prev_time:
            rospy.logwarn("IMU message is older than the previous message, ignoring it")
            return
        self.imu_2d.header.stamp = msg.header.stamp
        self.imu_2d.header.frame_id = msg.header.frame_id

        orientation = msg.orientation

        euler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        # convert euler angles to degrees
        euler = [x * 180 / 3.14159265359 for x in euler]
        
        print("Euler angles: ", euler[2])

class QuatToRPY:
    def __init__(self):
        self.imu_sub = rospy.Subscriber('pose', PoseStamped, self.callback)
        # self.imu_pub = rospy.Publisher('/imu_2d', Imu, queue_size=10)
        self.prev_time = rospy.Time.now()

    def callback(self, msg):

        orientation = msg.pose.orientation

        euler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        # convert euler angles to degrees
        euler = [x * 180 / 3.14159265359 for x in euler]
        
        print("Euler angles: ", euler[2])

if __name__ == '__main__':
    rospy.init_node('quat_to_rpy')
    try:
        # odom_2d = Odom2D()
        rpy = QuatToRPY()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass