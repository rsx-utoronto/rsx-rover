#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
from rover.msg import ArmInputs

# Based on arm_keyboard_controller.py / Adapted for GUI control

class GuiControllerNode():
    def __init__(self) -> None:
        rospy.init_node("arm_gui_controller")

        self.inputPublisher = rospy.Publisher("arm_inputs", ArmInputs, queue_size=10)
        self.statePubliser = rospy.Publisher("arm_state", String, queue_size=10)

    def on_press(self, command):
        GuiToController = ArmInputs()

        GuiToController.l_horizontal = 0
        GuiToController.l_vertical   = 0
        GuiToController.r_horizontal = 0
        GuiToController.r_vertical   = 0
        GuiToController.l1           = 0
        GuiToController.r1           = 0
        GuiToController.l2           = 0
        GuiToController.r2           = 0
        GuiToController.x            = 0
        GuiToController.o            = 0
        GuiToController.share        = 0
        GuiToController.options      = 0
        GuiToController.r3           = 0

        try:
            # left vertical joystick emulation
            if command == 'Forward':
                GuiToController.l_vertical = 1
            elif command == "Backward":
                GuiToController.l_vertical = -1

            # left horizontal joystick emulation
            if command == "Left":
                GuiToController.l_horizontal = 1
            elif command == "Righ":
                GuiToController.l_horizontal = -1

            # right vertical joystick emulation
            if command == 'Ry':
                GuiToController.r_vertical = 1
            elif command == "-Ry":
                GuiToController.r_vertical = -1

            # right horizontal joystick emulation
            if command == "Rz":
                GuiToController.r_horizontal = 1
            elif command == "-Rz":
                GuiToController.r_horizontal = -1

            # shape button emulation
            if command == "Temp: Open Grip":
                GuiToController.x = 1
            if command == "Temp: Close Grip":
                GuiToController.o = 1
            if command == "useless":
                GuiToController.triangle = 1
            if command == "useless":
                GuiToController.square = 1

            # other buttons
            if command == "Rx":
                GuiToController.l1 = 1
            if command == "-Rx":
                GuiToController.r1 = 1
            if command == "useless":
                GuiToController.share = 1
            if command == "useless":
                GuiToController.options = 1
            if command == "useless":
                GuiToController.r3 = 1
        except AttributeError:
            # left and right triggers
            if command == "Up":
                GuiToController.r2 = 1
            if command == "Down":
                GuiToController.l2 = 1

            # emulate d-pad as arrow keys
            if command == "Manual":
                self.statePubliser.publish("Manual")
            if command == "Inverse Kin":
                self.statePubliser.publish("IK")
            if command == "Setup":
                self.statePubliser.publish("Setup")
            if command == "Idle":
                self.statePubliser.publish("Idle")

        self.inputPublisher.publish(GuiToController)

    def on_release(self):
        keyboardToController = ArmInputs()

        keyboardToController.l_horizontal = 0
        keyboardToController.l_vertical   = 0
        keyboardToController.r_horizontal = 0
        keyboardToController.r_vertical   = 0
        keyboardToController.l1           = 0
        keyboardToController.r1           = 0
        keyboardToController.l2           = 0
        keyboardToController.r2           = 0
        keyboardToController.x            = 0
        keyboardToController.o            = 0
        keyboardToController.share        = 0
        keyboardToController.options      = 0
        keyboardToController.r3           = 0

        self.inputPublisher.publish(keyboardToController)
        
if __name__ == "__main__":
    # try:
    KeyInputNode = GuiControllerNode()
    # except Exception as ex:
        # print(ex)