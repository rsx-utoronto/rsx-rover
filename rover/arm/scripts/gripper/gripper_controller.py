#!/usr/bin/env python3

# Original Code by: Rudaina Khalil
# Latest Modification by: Abhay Verma (vabhay.1601@gmail.com)

import rospy
from rover.msg import ArmInputs
from std_msgs.msg import *
from arm_serial_connector import *

GRIPPER_CONVERSION = 32 # 32 encoder ticks result in 1 rotation of the gripper motor
GRIPPER_REDUCTION  = 30 # 30:1 Gear ratio on the gripper motor

class Gripper():
    """
    (str, str)

    This class represents all the functioning of the gripper motor.
    """

    def __init__(self, gripper_name= "STMicroelectronics_STM32_STLink_066DFF485671664867172828", servo_name= "1a86_USB2.0-Ser_"):
        
        ## Variables for serial connection
        # Gripper port
        self.gripper_connection = Serial_Port(device_name= gripper_name)
        self.gripper_connection.open_device_port(baudrate= 115200)

        # Servo controller port
        self.servo_connection = Serial_Port(device_name= servo_name)
        self.servo_connection.open_device_port(baudrate= 9600)

        # Device Connections
        # arduino_connection = Serial_Port(device_name= arduino_name)

        ## General variables
        # Start with "Idle"
        self.state               = 'Idle'

        # Variable for storing gripper_pos
        self.gripper_pos         = 0

        # Set speed limit for the gripper
        self.gripper_speed       = 1.5

        # Variable to to hold gripper roll positions (angles)
        self.gripper_roll        = 0

        ## Variables for ROS publishers and subscribers
        self.state_sub           = rospy.Subscriber("arm_state", String, self.CallbackState)
        self.input               = rospy.Subscriber("arm_inputs", ArmInputs, self.CallbackInput)
        self.SafePos_sub 		 = rospy.Subscriber("arm_safe_goal_pos", Float32MultiArray, self.CallbackSafePos)

    def CallbackState (self, state: String) -> None:
        """
        (String) -> (None)

        Recieves and stores state (idle, manual, ik or setup) from controller node

        @parameters

        state (String): stores the status received from the ROS topic
        """

        # Updates state
        self.state = state.data

    
    def CallbackInput (self, inputs: ArmInputs) -> None:
        """
        (ArmInputs) -> (None)

        Recieves inputs from controller and if state is not "Idle",
        updates the goal positions for the gripper and publishes them
        
        @parameters

        inputs (ArmInputs): Stores the received inputs from controller node
        """

        # Check if the gripper connection is successfull
        if not (self.gripper_connection.device_port and self.gripper_connection.device_port.is_open):
            print("Gripper motor controller not connected")
            return
        
        # Get inputs for the gripper and update the position
        gripper_input           = inputs.x - inputs.o
        self.gripper_pos        = gripper_input * self.gripper_speed + self.gripper_pos

        servo_config_flip       = inputs.square

        gripper_ticks           = int(self.gripper_pos / 360 * GRIPPER_CONVERSION * GRIPPER_REDUCTION)
        # Checking the state, only proceed if not in Idle
        if self.state != "Idle":

            # Send the new position
            self.gripper_connection.send_bytes(data= str(gripper_ticks) + '\n')
            
            if self.servo_connection.device_port:
                # TODO Add servo functionality here for future use
                pass


    def CallbackSafePos(self, data : Float32MultiArray):
        """
        (Float32MultiArray) -> (None)

        Callback funtion for receving SAFE_GOAL_POS data and updating the SAFE_GOAL_POS variable

        @parameters

        data (Float32MultiArray) : The data from the topic is stored in this parameter
        """
        # Get the data
        new_gripper_roll        = list(data.data)[-2]

        # Get the difference between current gripper goal pos and goal gripper pos
        difference          = new_gripper_roll - self.gripper_roll

        # Update the self.gripper_roll if changed
        if difference != 0:
            self.gripper_roll = new_gripper_roll
        
        # Set new gripper position
        self.gripper_pos    = self.gripper_pos + difference

        # Calculate the ticks for the motor
        gripper_ticks       = int((self.gripper_pos) / 360 * GRIPPER_CONVERSION * GRIPPER_REDUCTION)
        
        # Send the correction to the gripper motor
        self.gripper_connection.send_bytes(data= str(gripper_ticks) + '\n')


if __name__ == "__main__":
 
    try:
        rospy.init_node("Arm_Gripper")
        
        Gripper_Node = Gripper()

        rospy.spin()

        
    except rospy.ROSInterruptException:
        pass
    