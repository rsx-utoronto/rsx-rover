#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Imu
from transforms3d.euler import quat2euler
from geometry_msgs.msg import PoseStamped

class IMUQuatToRPY:
    def __init__(self):
        
        self.imu_sub= self.create_subscription(Imu, 'imu/orient', self.imu_callback, 10)
        # self.imu_pub = rospy.Publisher('/imu_2d', Imu, queue_size=10)
        self.prev_time = self.get_clock().now()
        self.imu_2d = Imu()

    def imu_callback(self, msg):
        if msg.header.stamp <= self.prev_time:
            rospy.logwarn("IMU message is older than the previous message, ignoring it")
            return
        self.imu_2d.header.stamp = msg.header.stamp
        self.imu_2d.header.frame_id = msg.header.frame_id

        orientation = msg.orientation

        euler = quat2euler([orientation.x, orientation.y, orientation.z, orientation.w])
        # convert euler angles to degrees
        euler = [x * 180 / 3.14159265359 for x in euler]
        
        print("Euler angles: ", euler[2])

class QuatToRPY(Node):
    def __init__(self):
        super().__init__('quat_to_rpy')
        
        self.imu_sub=self.create_subscription(PoseStamped, 'pose', self.callback, 10)
        # self.imu_pub = rospy.Publisher('/imu_2d', Imu, queue_size=10)
        self.prev_time = self.get_clock().now()

    def callback(self, msg):

        orientation = msg.pose.orientation

        euler = quat2euler([orientation.x, orientation.y, orientation.z, orientation.w])
        # convert euler angles to degrees
        euler = [x * 180 / 3.14159265359 for x in euler]
        
        print("Euler angles: ", euler[2])

if __name__ == '__main__':
    rclpy.init(args=None)
    try:
        # imu_2d = IMUQuatToRPY()
        quat_to_rpy = QuatToRPY()
        rclpy.spin(quat_to_rpy)
    except rclpy.exceptions.ROSInterruptException:
        pass
    finally:
        rclpy.shutdown()
    