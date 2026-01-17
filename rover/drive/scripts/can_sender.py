#!/usr/bin/python3

import math
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import can_util

class CanSenderNode(Node):
    def __init__(self, fl: int, fr: int, rl: int, rr: int, ff_torque: float):
        """
        Docstring for __init__
        
        :param fl: 6 bit CAN ID for front left wheel
        :type fl: int
        :param fr: 6 bit CAN ID for front right wheel
        :type fr: int
        :param rl: 6 bit CAN ID for rear left wheel
        :type rl: int
        :param rr: 6 bit CAN ID for rear right wheel
        :type rr: int
        :param ff_torque: Feedforward torque for the wheels in Nm
        :type ff_torque: float
        """
        super().__init__('can_sender_node')
        self.drive_sub = self.create_subscription(Twist, '/drive', self.drive_callback, 1)
        # self.vel_pub = self.create_publisher(Float32MultiArray, '/wheel_velocities', 1) # fl, fr, rl, rr
        self.bus = can_util.initialize_bus()
        self.WHEEL_BASE = 1.2
        self.FL_WHEEL_ID = (fl << 5) + 0xd
        self.FR_WHEEL_ID = (fr << 5) + 0xd
        self.RL_WHEEL_ID = (rl << 5) + 0xd
        self.RR_WHEEL_ID = (rr << 5) + 0xd
        self.WHEEL_CIR = 0.15 * 2 * math.pi
        self.FF_TORQUE = ff_torque
        self.get_logger().info('CAN Sender Node has been started.')

    def drive_callback(self, msg: Twist):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        left_wheel_velocity = linear_velocity / self.WHEEL_CIR - (angular_velocity * self.WHEEL_BASE / 2) # rev/s
        right_wheel_velocity = linear_velocity / self.WHEEL_CIR + (angular_velocity * self.WHEEL_BASE / 2) # rev/s

        # Pack data: vel (float32) at byte 0, torque (float32) at byte 4
        left_wheel_data = struct.pack('<ff', left_wheel_velocity, self.FF_TORQUE)
        right_wheel_data = struct.pack('<ff', right_wheel_velocity, self.FF_TORQUE)

        # Here you would add the code to send the velocities over CAN bus.
        can_util.send_can_message(self.bus, self.FL_WHEEL_ID, left_wheel_data)
        can_util.send_can_message(self.bus, self.FR_WHEEL_ID, right_wheel_data)
        can_util.send_can_message(self.bus, self.RL_WHEEL_ID, left_wheel_data)
        can_util.send_can_message(self.bus, self.RR_WHEEL_ID, right_wheel_data)
        # For demonstration, we will just log the values.
        self.get_logger().info(f'Received drive command - Linear Velocity: {linear_velocity}, Angular Velocity: {angular_velocity}')
        
    # def publish_wheel_velocities(self):
    #     data = self.bus.recv(0.1)  # Non-blocking receive with timeout
    #     if self.bus is None:
    #         return
    #     wheel_velocities = Float32MultiArray()
        

def main(args=None):
    rclpy.init(args=args)
    can_sender_node = CanSenderNode(fl=1, fr=2, rl=3, rr=4, ff_torque=0.5)
    rclpy.spin(can_sender_node)
    can_sender_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()