#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
from rover.msg import ArmInputs


# Based on arm_keyboard_controller.py / Adapted for GUI control

class GuiControllerNode():
    def __init__(self) -> None:
        rospy.init_node("arm_gui_controller")

        self.inputPublisher = rospy.Publisher("arm_inputs", ArmInputs, queue_size=10)
        self.statePublisher = rospy.Publisher("arm_state", String, queue_size=10)
        
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

        speedMultiplier = 6900
        try:
            # left vertical joystick emulation
            if command == "Forward" or command == "joint1plus":
                GuiToController.l_vertical = 1 * speedMultiplier
            elif command == "Backward" or command == "joint1minus":
                GuiToController.l_vertical = -1 * speedMultiplier

            # left horizontal joystick emulation
            if command == "Left" or command == "joint0plus":
                GuiToController.l_horizontal = 1 * speedMultiplier
            elif command == "Right" or command == "joint0minus":
                GuiToController.l_horizontal = -1 * speedMultiplier

            # left and right triggers
            if command == "Up" or command == "joint5plus":
                GuiToController.r2 = 1 * speedMultiplier
            if command == "Down" or command == "joint5minus":
                GuiToController.l2 = 1 * speedMultiplier

            # Rx
            if command == "Rx" or command == "joint4plus":
                GuiToController.l1 = 1 * speedMultiplier
            elif command == "-Rx" or command == "joint4minus":
                GuiToController.r1 = 1 * speedMultiplier
            
            # right vertical joystick emulation
            if command == "Ry" or command == "joint2plus":
                GuiToController.r_vertical = 1 * speedMultiplier
            elif command == "-Ry" or command == "joint2minus":
                GuiToController.r_vertical = -1 * speedMultiplier

            # right horizontal joystick emulation
            if command == "Rz" or command == "joint3plus":
                GuiToController.r_horizontal = 1 * speedMultiplier
            elif command == "-Rz" or command == "joint3minus":
                GuiToController.r_horizontal = -1 * speedMultiplier

            # shape button emulation
            if command == "Open Grip":
                GuiToController.x = 1 
            if command == "Close Grip":
                GuiToController.o = 1
            if command == "useless":
                GuiToController.triangle = 1
            if command == "useless":
                GuiToController.square = 1

            # other buttons
            if command == "useless":
                GuiToController.share = 1
            if command == "useless":
                GuiToController.options = 1
            if command == "useless":
                GuiToController.r3 = 1
                
            # emulate d-pad as arrow keys
            if command == "Manual":
                self.statePublisher.publish("Manual")
            if command == "Inverse Kin":
                self.statePublisher.publish("IK")
            if command == "Setup":
                self.statePublisher.publish("Setup")
            if command == "Idle":
                self.statePublisher.publish("Idle")
        except:
            print("error!")

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