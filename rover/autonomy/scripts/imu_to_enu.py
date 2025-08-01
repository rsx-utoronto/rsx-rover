#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from transforms3d.euler import quat2euler, euler2quat


class ImuDataToENU(Node):
    def __init__(self):
        super().__init__('imu_to_enu')
        # self.sub = rospy.Subscriber('/zed_node/imu/data', Imu, self.imu_callback)
        # self.pub = rospy.Publisher('/imu/enu', Imu, queue_size=1)
        # rospy.spin()
        self.sub= self.create_subscription(Imu, '/zed_node/imu/data', self.imu_callback, 10)
        self.pub = self.create_publisher(Imu, '/imu/enu', 1)
        

    def imu_callback(self, data):
        transformed = Imu()
        transformed.header.stamp = data.header.stamp
        transformed.header.frame_id = 'base_link'
        transformed.linear_acceleration.x = -data.linear_acceleration.y
        transformed.linear_acceleration.y = data.linear_acceleration.x
        transformed.linear_acceleration.z = data.linear_acceleration.z
        transformed.angular_velocity.x = -data.angular_velocity.y
        transformed.angular_velocity.y = data.angular_velocity.x
        transformed.angular_velocity.z = data.angular_velocity.z
        new_orientation = self.transform_orientation(data.orientation)
        transformed.orientation.x = new_orientation[0]
        transformed.orientation.y = new_orientation[1]
        transformed.orientation.z = new_orientation[2]
        transformed.orientation.w = new_orientation[3]
        transformed.linear_acceleration_covariance = self.transform_covariance(data.linear_acceleration_covariance)
        transformed.angular_velocity_covariance = self.transform_covariance(data.angular_velocity_covariance)
        transformed.orientation_covariance = self.transform_covariance(data.orientation_covariance)
        self.pub.publish(transformed)

    def transform_orientation(self, orientation):
        rpy = quat2euler([orientation.x, orientation.y, orientation.z, orientation.w])
        new_orientation = euler2quat(-rpy[1], rpy[0], rpy[2])
        return new_orientation

    def transform_covariance(self, covariance):
        new_covariance = [0] * 9
        new_covariance[0] = covariance[4]
        new_covariance[1] = covariance[3]
        new_covariance[2] = covariance[5]
        new_covariance[3] = covariance[1]
        new_covariance[4] = covariance[0]
        new_covariance[5] = covariance[2]
        new_covariance[6] = covariance[7]
        new_covariance[7] = covariance[6]
        new_covariance[8] = covariance[8]
        return new_covariance

if __name__ == '__main__':
    rclpy.init(args=None)
    try:
        imu_to_enu = ImuDataToENU()
        rclpy.spin(imu_to_enu)
    except rclpy.exceptions.ROSInterruptException:
        pass
    finally:
        rclpy.shutdown()
    