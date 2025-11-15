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
        self.manual_script_path = '/home/rsx/rover_ws/src/rsx-rover/scripts/manual_auto'
        self.switch_pressed = False
        # keep previous buttons state to detect rising edges (press events)
        self.prev_buttons = None
        # self.PS = 0
        # print("Waiting for the switch to be pressed")
        # self.L1 = 0
        # self.R1 = 0
    def callback(self, msg):
        # store common buttons for compatibility but guard against short arrays
        self.R1 = msg.buttons[5] if len(msg.buttons) > 5 else 0
        self.L1 = msg.buttons[4] if len(msg.buttons) > 4 else 0

        # initialize previous buttons on first message
        if self.prev_buttons is None:
            self.prev_buttons = list(msg.buttons)

        # detect rising edges (0 -> 1) and print which button index was pressed
        for idx, val in enumerate(msg.buttons):
            prev = self.prev_buttons[idx] if idx < len(self.prev_buttons) else 0
            if val == 1 and prev == 0:
                print(f"Button pressed index: {idx}")
                # if it's the PS index you expect (10) execute the script
                if idx == 12:
                    print("PS (index 10) detected: running manual script")
                    try:
                        result = subprocess.run(['bash', self.manual_script_path], check=True, capture_output=True, text=True)
                        print("Script output:\n", result.stdout)
                    except subprocess.CalledProcessError as e:
                        print(f"Script failed with error: {e.stderr}")
                    self.switch_pressed = True

        # update previous buttons snapshot
        self.prev_buttons = list(msg.buttons)
        # check if zed publishing, if it doesn't have anything for x time wait 5 seconds check, if still no, relaunch the zed_dies.
  
if __name__ == '__main__':
        rclpy.init(args=None)
        manual_switch = ManualSwitch()
        rclpy.spin(manual_switch)
        manual_switch.destroy_node()
        rclpy.shutdown()
   
