#!/usr/bin/env python3

import rospy
from std_msgs.msg import *
from enum import Enum
import time
import numpy as np
import os

############ ENUMERATIONS #############

# Enum for all the errors
class Errors(Enum):
    ERROR_NONE              = 0
    ERROR_EXCEEDING_POS     = 1
    ERROR_EXCEEDING_CURRENT = 2
    ERROR_LIMIT_SWITCH      = 3

############### CLASSES ###############

class Safety_Node():
    """
    (None)

    This class represents an instance of the safety node and connects the ROS node to
    multiple topics by making it a subscriber or a publisher
    """

    def __init__(self):

        # Attributes to hold data from subscribed topics
        self.GOAL_POS             = [0, 0, 0, 0, 0, 0, 0]
        self.CURR_POS             = [0, 0, 0, 0, 0, 0, 0]
        self.MOTOR_CURR           = [0, 0, 0, 0, 0, 0, 0]
        self.LIMIT_SWITCH         = [False, False, False, False, False, False, False]

        # Attributes to hold data for publishing to topics
        self.ERRORS               = UInt8MultiArray()
        self.ERRORS.data          = [0, 0, 0, 0, 0, 0, 0]
        self.SAFE_GOAL_POS        = Float32MultiArray()
        self.SAFE_GOAL_POS.data   = [0, 0, 0, 0, 0, 0, 0]
        self.ERROR_OFFSET         = Float32MultiArray()
        self.ERROR_OFFSET.data    = [0, 0, 0, 0, 0, 0, 0]

        # Attributes needed for detecting whether motor current is exceeding
        self.TIME                 = [0, 0, 0, 0, 0, 0, 0]
        self.FIRST                = [True, True, True, True, True, True, True]

        # Variables for ROS publishers and subscribers
        self.Goal_sub             = rospy.Subscriber("arm_goal_pos", Float32MultiArray, self.callback_Goal)
        self.MotorCurr_sub        = rospy.Subscriber("arm_motor_curr", Float32MultiArray, self.callback_MotorCurr)
        self.CurrPos_sub          = rospy.Subscriber("arm_curr_pos", Float32MultiArray, self.callback_CurrPos)
        self.LimitSwitch_sub      = rospy.Subscriber("arm_limit_switch", UInt8MultiArray, self.callback_LimitSwitch)
        self.KillSwitch_sub       = rospy.Subscriber("arm_killswitch", UInt8, self.callback_KillSwitch)
        self.SafePos_pub          = rospy.Publisher("arm_safe_goal_pos", Float32MultiArray, queue_size= 0)
        self.Error_pub            = rospy.Publisher("arm_error_msg", UInt8MultiArray, queue_size= 0)
        self.Offset_pub           = rospy.Publisher("arm_error_offset", Float32MultiArray, queue_size= 0)

    def callback_KillSwitch(self, data : UInt8):
        """
        (UInt8) -> (None)
        
        Receives and stores killswitch input from the controller node. If pressed, we shut down CAN_Send node
        after sending the curr_pos as the latest position.

        @parameters

        data (UInt8): Input indicating whether killswitch is pressed or not
        """
        
        # Storing the boolean value
        killswitch = data.data

        if killswitch:
            self.SAFE_GOAL_POS.data = self.CURR_POS
            self.SafePos_pub.publish(self.SAFE_GOAL_POS)
            rospy.sleep(0.001)
            os.system("rosnode kill "+ "CAN_Send")
        
        else:
            os.system("rosrun rover "+ "CAN_send.py")
            self.SAFE_GOAL_POS.data = self.CURR_POS


    def callback_LimitSwitch(self, limitSwitch_data : UInt8MultiArray) -> None:
        """
        (UInt8MultiArray) -> (None)
        
        Receives and stores limit switch inputs from SparkMax through the LIMIT_SWITCH topic
        into LIMIT_SWITCH attribute. Also updates the ERRORS attribute if needed and publishes it on 
        ERROR_MSG topic

        @parameters

        limitSwitch_data (UInt8MultiArray): List containing limit switch inputs from SparkMax
        """

        # Store the received limit switch data
        self.LIMIT_SWITCH = limitSwitch_data.data
        #print(self.LIMIT_SWITCH)

        # Updating the ERRORS attribute if needed and publishing it
        self.limitSwitch_check()
        self.Error_pub.publish(self.ERRORS)

    def callback_Goal(self, goal_data : Float32MultiArray) -> None:
        """
        (Float32MultiArray) -> (None)

        Receives and stores data from GOAL_POS topic into GOAL_POS attribute, checks
        if the value is safe and publishes it to the SAFE_GOAL_POS topic if it is safe

        After the safety check, also publishes the errors to ERROR_MSG topic

        @parameters

        goal_data (Float32MultiArray): List containing input angles (degrees) of each motor
        """

        # Store the received goal postion
        self.GOAL_POS = list(goal_data.data)

        # Check if the goal position is safe and publish the errors
        #self.postion_check()
        #self.Error_pub.publish(self.ERRORS)

        # Check if there are any other errors
        if self.ERRORS.data.count(Errors.ERROR_NONE.value) == len(self.ERRORS.data):

            # Print/Publish the position to SAFE_GOAL_POS topic
            self.SAFE_GOAL_POS.data = self.GOAL_POS
            self.SafePos_pub.publish(self.SAFE_GOAL_POS)
        
        # If there are any errors, publish the error offsets and reset them
        else:
            self.Offset_pub.publish(self.ERROR_OFFSET)

            #self.ERROR_OFFSET.data  = [0, 0, 0, 0, 0, 0, 0]

    
    def callback_MotorCurr(self, MotorCurr_data : Float32MultiArray) -> None:
        """
        (Float32MultiArray) -> (None)

        Receives and stores data from MOTOR_CURR topic into MOTOR_CURR attribute and
        checks whether any current value is exceeding limits. Publishes the errors on 
        ERROR_MSG topic after the checks

        @parameters

        MotorCurr_data (Float32MultiArray): List containing current values (amps, I think) of each motor
        """
        
        # Store the received motor current value
        self.MOTOR_CURR = MotorCurr_data.data

        # Check if motor current is exceeding and publish the errors
        self.current_check()
        self.Error_pub.publish(self.ERRORS)
    
    def callback_CurrPos(self, CurrPos_data : Float32MultiArray) -> None:
        """
        (Float32MultiArray) -> (None)

        Receives and stores data from CURR_POS topic into CURR_POS attribute

        @parameters

        CurrPos_data (Float32MultiArray): List containing current angles (degrees) of each motor
        """
        self.CURR_POS = CurrPos_data.data
    
    def postion_check(self) -> None:
        '''
        (float) -> (None)

        Compares the goal position with current position to see that the difference 
        between them is not gigantic. If gigantic sets the error as for that motor as
        Errors.ERROR_EXCEEDING_POS. Makes sure the motor doesn't jerk back to 0 
        position in case computer loses power
        '''
        # TODO
        # Limits for position safety (Need to test these values)
        limit = [1, 1, 1, 1, 1, 1, 1]

        # Going through each element of GOAL_POS
        #for i in range(len(self.GOAL_POS)):
        for i in [0, 1, 2]:    
            # Doing position comparisons for safety
            if (self.GOAL_POS[i] < (self.CURR_POS[i] - limit[i]) or 
                self.GOAL_POS[i] > (self.CURR_POS[i] + limit[i])):

                # Calculating the offset, applying it to goal position and storing it
                offset                  = list(np.array(self.GOAL_POS) - np.array(self.CURR_POS))
                self.ERROR_OFFSET.data  = offset
                #spark_input[i] = arm_can.pos_to_sparkdata(CURR_POS[i])

                # Set the error for the motor
                self.ERRORS.data[i]     = Errors.ERROR_EXCEEDING_POS.value

            # NOTE
            # Remove the error if it is no longer there
            # By design, the EXCEEDING_POS error should not be there after one 
            # cycle of the loop as the offset to controller input will bring
            # the input back to a safe value.
            else:
                if (self.ERRORS.data[i] == Errors.ERROR_NONE.value or 
                    self.ERRORS.data[i] == Errors.ERROR_EXCEEDING_POS.value):
                    self.ERRORS.data[i] = Errors.ERROR_NONE.value


    def current_check(self) -> None:
        '''
        (None) -> (None)

        Checks the maximum current being consumed by a motor. If the current is higher than
        the expected max, sets the error for the particular motor as Errors.ERROR_EXCEEDING_CURRENT.
        Helps us to know when too much torque is being applied
        '''

        # Max current values for each motor
        max_current = [40, 40, 40, 20, 20, 20, 20]

        # Checking maximum currents
        for i in range(len(self.MOTOR_CURR)):
            if self.MOTOR_CURR[i] > max_current[i]:

                # Checking if the input from controller is making it go further into 
                # the direction that increases current
                if (sign(self.GOAL_POS[i] - self.CURR_POS[i]) == sign(self.GOAL_POS[i])):
                    #spark_input[i] = arm_can.pos_to_sparkdata(CURR_POS[i])
                    # Check if the motor has an error associated with it already
                    if (self.ERRORS.data[i] == Errors.ERROR_NONE.value or 
                        self.ERRORS.data[i] == Errors.ERROR_EXCEEDING_CURRENT.value):

                        # Change the error associated with the motor to EXCEEDING_CURRENT
                        self.ERRORS.data[i] = Errors.ERROR_EXCEEDING_CURRENT.value

                        # Calculating the offset, applying it to goal position and storing it
                        offset                  = list(np.array(self.GOAL_POS) - np.array(self.CURR_POS))
                        self.ERROR_OFFSET.data  = offset

                        # Check if this is the first time this error is set 
                        if self.FIRST[i]:

                            # Collect time stamp since the error was set
                            self.TIME[i]  = time.time()
                            self.FIRST[i] = False
                
                # If the controller input is in the direction that reduces the current
                else:
                    # Check if the motor has an associated with it already
                    if (self.ERRORS.data[i] == Errors.ERROR_NONE.value or 
                        self.ERRORS.data[i] == Errors.ERROR_EXCEEDING_CURRENT.value):

                        # Remove any error and set TIME and FIRST back to original form
                        self.ERRORS.data[i] = Errors.ERROR_NONE.value
                        self.FIRST[i]  = True
                        self.TIME[i]   = 0
            
            # Checking if the current has been under the max limit for more than 10 ms
            elif self.MOTOR_CURR[i] < max_current[i] and time.time() - self.TIME[i] > 0.01:

                # Check if the motor has an error associated with it already
                if (self.ERRORS.data[i] == Errors.ERROR_EXCEEDING_CURRENT.value or
                    self.ERRORS.data[i] == Errors.ERROR_NONE.value):

                    # Remove the error and set TIME and FIRST back to original form
                    self.ERRORS.data[i] = Errors.ERROR_NONE.value
                    self.FIRST[i]  = True
                    self.TIME[i]   = 0

    def limitSwitch_check(self) -> None:
        '''
        (None) -> (None)

        Checks whether any motor of the arm is hitting the limit switch attached to it. 
        Updates the error status of the motor if needed
        '''
        # Going through each limit switch input
        for i in range(len(self.LIMIT_SWITCH)):

            # Checking if limit switch is being pressed, 
            # if yes then change the error status for the motor
            if self.LIMIT_SWITCH[i] == True:
                self.ERRORS.data[i]     = Errors.ERROR_LIMIT_SWITCH.value

                # Calculating the offset, applying it to goal position and storing it
                offset                  = list(np.array(self.GOAL_POS) - np.array(self.CURR_POS))
                print(self.GOAL_POS[2], self.CURR_POS[2], offset[2])
                self.ERROR_OFFSET.data  = offset

            else:

                # Checking if any other error is set already, if not then change it back no error
                if (self.ERRORS.data[i] == Errors.ERROR_NONE.value or 
                    self.ERRORS.data[i] == Errors.ERROR_LIMIT_SWITCH.value):

                    self.ERRORS.data[i] = Errors.ERROR_NONE.value

    def setup(self):
        # For each motor...
        for i in range(len(self.SAFE_GOAL_POS) - 1):
            # Store current angles
            current = self.CURR_POS[i]
            # Send in a large value to hit limit switch
            self.SAFE_GOAL_POS[i] = 10000000
            # When an error occurs, check again and reverse angle change
            while Errors.ERRORS_LIMIT_SWITCH.value != self.ERRORS.data[i]:
                pass
            final = self.CURR_POS[i]
            self.CURR_POS[i] = final - (final - current)



############## FUNCTIONS ##############

def sign(x : float) -> int:
	'''
	(float) -> (int)

	Returns the sign of the given number

	@parameters

	x (float) = The number for which sign is needed
	'''

	return (x > 0) - (x < 0)

###################### MAIN #########################

def main() -> None:
    '''
    (None) -> (None)
    
    Main function that initializes the safety node
    '''

    try:
        # Initialize a ROS node
        rospy.init_node("Arm_Safety")

        # Set all the publishers and subscribers of the node
        Safety = Safety_Node()

        # ROS spin to keep the node alive
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":

    main()