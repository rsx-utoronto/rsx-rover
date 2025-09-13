#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import subprocess


class ManualSwitch(Node):
    def __init__(self):
        super().__init__('manual_switch_node')
      
       
        sub=self.create_subscription(
            Joy,
            '/auto/joy',
            self.callback,
            10
        )
        self.manual_script_path = '/home/rsx/ros2_ws/src/rsx-rover/scripts/manual_auto'
        self.switch_pressed = False
        # self.PS = 0
        # print("Waiting for the switch to be pressed")
        # self.L1 = 0
        # self.R1 = 0
    def callback(self, msg):
        self.R1 = msg.buttons[5]
        self.L1 = msg.buttons[4]
        self.PS = msg.buttons[10]
        print("PS: ", self.PS)
        if self.PS == 1:
            # Use subprocess.run to execute the bash script
            print("Switch pressed")
            try:
                result = subprocess.run(['bash', self.manual_script_path], check=True, capture_output=True, text=True)
                print("Script output:\n", result.stdout)
            except subprocess.CalledProcessError as e:
                print(f"Script failed with error: {e.stderr}")
            self.switch_pressed = True
        # check if zed publishing, if it doesn't have anything for x time wait 5 seconds check, if still no, relaunch the zed_dies.
  
if __name__ == '__main__':
        rclpy.init(args=None)
        manual_switch = ManualSwitch()
        rclpy.spin(manual_switch)
        manual_switch.destroy_node()
        rclpy.shutdown()
   
