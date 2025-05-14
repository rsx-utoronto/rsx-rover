#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
import subprocess


class ManualSwitch:
    def __init__(self):
        rospy.init_node('auto_switch')
        sub = rospy.Subscriber('/auto/joy', Joy, self.callback)
        self.manual_script_path = '/home/rsx/rover_ws/src/rsx-rover/scripts/manual_auto'
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
    while not rospy.is_shutdown():
        manual_switch = ManualSwitch()
        if manual_switch.switch_pressed:
            break
        rospy.spin()

