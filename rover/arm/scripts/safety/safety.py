import arm_can_control as arm_can
from GetManualJoystickFinal import *
from MapManualJoystick import *
#import arm_servo as servo
import struct
import rospy
from std_msgs.msg import Float32MultiArray
import threading
from enum import Enum

############ ENUMERATIONS #############

# Enum for all the errors
class Errors(Enum):
    ERROR_NONE              = 0
    ERROR_EXCEEDING_POS     = 1
    ERROR_EXCEEDING_CURRENT = 2
    ERROR_LIMIT_SWITCH      = 3

########### GLOBAL CONSTANTS ###########

REDUCTION       = [120, 160, 120, 20, 20, 20, 40]


############### CLASSES ###############

class Safety_Node():
    """
    (None)

    This class represents an instance of the safety node and connects the ROS node to
    multiple topics by making it a subscriber or a publisher
    """

    def __init__(self):

        # Attributes to hold data from subscribed topics
        self.GOAL_POS        = [0, 0, 0, 0, 0, 0, 0]
        self.CURR_POS        = [0, 0, 0, 0, 0, 0, 0]
        self.MOTOR_CURR      = [0, 0, 0, 0, 0, 0, 0]
        self.LIMIT_SWITCH    = [False, False, False, False, False, False, False]

        # Attributes to hold data for publishing to topics
        self.ERRORS          = [0, 0, 0, 0, 0, 0, 0]

        # Attributes needed for detecting whether motor current is exceeding
        self.TIME            = [0, 0, 0, 0, 0, 0, 0]
        self.FIRST           = [True, True, True, True, True, True, True]

        # Variables for ROS publishers and subscribers
        self.Goal_sub        = rospy.Subscriber("GOAL_POS", Float32MultiArray, self.callback_Goal)
        self.MotorCurr_sub   = rospy.Subscriber("MOTOR_CURR", Float32MultiArray, self.callback_MotorCurr)
        self.CurrPos_sub     = rospy.Subscriber("CURR_POS", Float32MultiArray, self.callback_CurrPos)
        self.LimitSwitch_sub = rospy.Subscriber("LIMIT_SWITCH", Float32MultiArray, self.callback_LimitSwitch)
        self.SafePos_pub     = rospy.Publisher("SAFE_GOAL_POS", Float32MultiArray, queue_size= 10)
        self.Error_pub       = rospy.Publisher("ERROR_MSG", Float32MultiArray, queue_size= 10)

    def callback_LimitSwitch(self, limitSwitch_data : list(int)) -> None:
        """
        list(int) -> (None)
        
        Receives and stores limit switch inputs from SparkMax through the LIMIT_SWITCH topic
        into LIMIT_SWITCH attribute. Also updates the ERRORS attribute if needed and publishes it on 
        ERROR_MSG topic

        @parameters

        limitSwitch_data (list(int)): List containing limit switch inputs from SparkMax
        """

        # Store the received limit switch data
        self.LIMIT_SWITCH = limitSwitch_data.data

        # Updating the ERRORS attribute if needed and publishing it
        self.limitSwitch_check()
        self.Error_pub.publish(self.ERRORS)

    def callback_Goal(self, goal_data : list(float)) -> None:
        """
        (list(float)) -> (None)

        Receives and stores data from GOAL_POS topic into GOAL_POS attribute, checks
        if the value is safe and publishes it to the SAFE_GOAL_POS topic if it is safe

        After the safety check, also publishes the errors to ERROR_MSG topic

        @parameters

        goal_data (list(float)): List containing input angles (degrees) of each motor
        """

        # Store the received goal postion
        self.GOAL_POS = goal_data.data

        # Check if the goal position is safe and publish the errors
        self.postion_check()
        self.Error_pub.publish(self.ERRORS)

        # Check if there are any other errors
        if self.ERRORS.count(Errors.ERROR_NONE) == len(self.ERRORS):

            # Publish the position to SAFE_GOAL_POS topic
            self.SafePos_pub.publish(self.GOAL_POS)
    
    def callback_MotorCurr(self, MotorCurr_data : list(float)) -> None:
        """
        (list(float)) -> (None)

        Receives and stores data from MOTOR_CURR topic into MOTOR_CURR attribute and
        checks whether any current value is exceeding limits. Publishes the errors on 
        ERROR_MSG topic after the checks

        @parameters

        MotorCurr_data (list(float)): List containing current values (amps, I think) of each motor
        """
        
        # Store the received motor current value
        self.MOTOR_CURR = MotorCurr_data.data

        # Check if motor current is exceeding and publish the errors
        self.current_check()
        self.Error_pub.publish(self.ERRORS)


    
    def callback_CurrPos(self, CurrPos_data : list(float)) -> None:
        """
        (list(float)) -> (None)

        Receives and stores data from CURR_POS topic into CURR_POS attribute

        @parameters

        CurrPos_data (list(float)): List containing current angles (degrees) of each motor
        """
        self.CURR_POS = CurrPos_data.data
    
    def postion_check(self, dt : float = 0.1) -> None:
        '''
        (float) -> (None)

        Compares the goal position with current position to see that the difference 
        between them is not gigantic. If gigantic sets the error as for that motor as
        Errors.ERROR_EXCEEDING_POS. Makes sure the motor doesn't jerk back to 0 
        position in case computer loses power

        @parameters

        dt (float) (optional): Time value (in s) to be multipled with speed limits
        '''

        # Multiplying factors for position safety
        factor = [0.6, 0.8, 0.15, 0.05, 1/15, 1/15, 1/50]

        # Going through each element of GOAL_POS
        for i in range(len(self.GOAL_POS)):
            
            # Starting thread lock
            #with lock:
            #print(dt)
            # Doing position comparisons for safety
            if self.GOAL_POS[i] < (self.CURR_POS[i] - factor[i] * speed_limit[i] * dt) or self.GOAL_POS[i] > (self.CURR_POS[i] + factor[i] * speed_limit[i] * dt):

                # Calculating the offset and applying it to controller input
                offset            = self.GOAL_POS[i] - self.CURR_POS[i]
                self.GOAL_POS[i] -= offset
                #spark_input[i] = arm_can.pos_to_sparkdata(CURR_POS[i])

                # Set the error for the motor
                self.ERRORS[i]    = Errors.ERROR_EXCEEDING_POS

                # For debugging safety code, you can uncomment this
                #spark_input[i] = arm_can.pos_to_sparkdata(CURR_POS[i])

            # Remove the error if it is no longer there
            # By design, the EXCEEDING_POS error should not be there after one 
            # cycle of the main loop as the offset to controller input will bring
            # the input back to a safe value.
            else:
                if self.ERRORS[i] == Errors.ERROR_NONE or self.ERRORS[i] == Errors.ERROR_EXCEEDING_POS:
                    self.ERRORS[i] = Errors.ERROR_NONE

        #print("Input:", arm_can.read_can_message(spark_input[4], arm_can.CMD_API_STAT2))

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
                    if self.ERRORS[i] == Errors.ERROR_NONE or self.ERRORS[i] == Errors.ERROR_EXCEEDING_CURRENT:

                        # Change the error associated with the motor to EXCEEDING_CURRENT
                        self.ERRORS[i] = Errors.ERROR_EXCEEDING_CURRENT

                        # Check if this is the first time this error is set 
                        if self.FIRST[i]:

                            # Collect time stamp since the error was set
                            self.TIME[i]  = time.time()
                            self.FIRST[i] = False
                
                # If the controller input is in the direction that reduces the current
                else:
                    # Check if the motor has an associated with it already
                    if self.ERRORS[i] == Errors.ERROR_NONE or self.ERRORS[i] == Errors.ERROR_EXCEEDING_CURRENT:

                        # Remove any error and set TIME and FIRST back to original form
                        self.ERRORS[i] = Errors.ERROR_NONE
                        self.FIRST[i]  = True
                        self.TIME[i]   = 0
            
            # Checking if the current has been under the max limit for more than 10 ms
            elif self.MOTOR_CURR[i] < max_current[i] and time.time() - self.TIME[i] > 0.01:

                # Check if the motor has an error associated with it already
                if self.ERRORS[i] != Errors.ERROR_EXCEEDING_POS:

                    # Remove the error and set TIME and FIRST back to original form
                    self.ERRORS[i] = Errors.ERROR_NONE
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

            # Checking if any other error is set already
            if self.ERRORS[i] == Errors.ERROR_NONE or self.ERRORS[i] == Errors.ERROR_LIMIT_SWITCH:

                # Checking if limit switch is being pressed, 
                # if yes then change the error status for the motor
                if self.LIMIT_SWITCH[i] == True:
                    self.ERRORS[i] = Errors.ERROR_LIMIT_SWITCH


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
    # Initialize a ROS node
    rospy.init_node("Arm_Safety", anonymous= True)

    # Set all the publishers and subscribers of the node
    Safety = Safety_Node()

    # ROS spin to keep the node alive
    rospy.spin()

if __name__ == "__main__":
    main()

# def postion_check(spark_input : list, dt : float = 0.1) -> list(int):
#     '''
#     (list(int)) -> (list(int))

#     Performs safety operations:
#     1) Compares the goal position with current position to see that the difference 
#     between them is not gigantic. If gigantic sets the error as for that motor as
#     Errors.ERROR_EXCEEDING_POS. Makes sure the motor doesn't jerk back to 0 
#     position in case computer loses power

#     2) Checks the maximum current being consumed by a motor. If the current is higher than
#     the expected max, sets the error for the particular motor as Errors.ERROR_EXCEEDING_CURRENT.
#     Helps us to know when too much torque is being applied

#     @parameters

#     spark_input (list(int)): Input positions for the motors
#     '''

#     # Multiplying factors for position safety
#     factor = [0.6, 0.8, 0.15, 0.05, 1/15, 1/15, 1/50]

#     # Checking if input is as long as CURR_POS
#     if len(spark_input) == len(CURR_POS):

#         for i in range(len(spark_input)):
            
#             # Starting thread lock
#             #with lock:
#             #print(dt)
#             # Doing position comparisons for safety
#             if GOAL_POS[i] < (CURR_POS[i] - factor[i] * speed_limit[i] * 0.1) or GOAL_POS[i] > (CURR_POS[i] + factor[i] * speed_limit[i] * 0.1):

#                 # Calculating the offset and applying it to controller input
#                 offset = GOAL_POS[i] - CURR_POS[i]
#                 GOAL_POS[i] -= offset
#                 #spark_input[i] = arm_can.pos_to_sparkdata(CURR_POS[i])

#                 # Set the error for the motor
#                 ERRORS[i] = Errors.ERROR_EXCEEDING_POS

#                 # For debugging safety code, you can uncomment this
#                 #spark_input[i] = arm_can.pos_to_sparkdata(CURR_POS[i])

#             # Remove the error if it is no longer there
#             # By design, the EXCEEDING_POS error should not be there after one 
#             # cycle of the main loop as the offset to controller input will bring
#             # the input back to a safe value.
#             else:
#                 if ERRORS[i] != Errors.ERROR_EXCEEDING_CURRENT:
#                     ERRORS[i] = Errors.ERROR_NONE

#         #print("Input:", arm_can.read_can_message(spark_input[4], arm_can.CMD_API_STAT2))
#         return spark_input
        
#     else:
#         print('ERROR: "spark_input" is invalid, returning CURR_POS')

# def current_check(spark_input : list, dt : float = 0.1):
#     '''
#     (list(int)) -> (list(int))

#     Performs safety operations:
#     1) Compares the goal position with current position to see that the difference 
#     between them is not gigantic. If gigantic sets the error as for that motor as
#     Errors.ERROR_EXCEEDING_POS. Makes sure the motor doesn't jerk back to 0 
#     position in case computer loses power

#     2) Checks the maximum current being consumed by a motor. If the current is higher than
#     the expected max, sets the error for the particular motor as Errors.ERROR_EXCEEDING_CURRENT.
#     Helps us to know when too much torque is being applied

#     @parameters

#     spark_input (list(int)): Input positions for the motors
#     '''

#     # Max current values for each motor
#     max_current = [40, 40, 40, 20, 20, 20, 20]

#     # Checking maximum currents
#     for i in range(len(MOTOR_CURR)):
#         if MOTOR_CURR[i] > max_current[i]:

#             # Checking if the input from controller is making it go further into 
#             # the direction that increases current
#             if (sign(GOAL_POS[i] - CURR_POS[i]) == sign(GOAL_POS[i])):
#                 #spark_input[i] = arm_can.pos_to_sparkdata(CURR_POS[i])
#                 # Check if the motor has an error associated with it already
#                 if ERRORS[i] != Errors.ERROR_EXCEEDING_POS:

#                     # Change the error associated with the motor to EXCEEDING_CURRENT
#                     ERRORS[i] = Errors.ERROR_EXCEEDING_CURRENT

#                     # Check if this is the first time this error is set 
#                     if FIRST[i]:

#                         # Collect time stamp since the error was set
#                         TIME[i]  = time.time()
#                         FIRST[i] = False
            
#             # If the controller input is in the direction that reduces the current
#             else:
#                 # Check if the motor has an associated with it already
#                 if ERRORS[i] != Errors.ERROR_EXCEEDING_POS:

#                     # Remove any error and set TIME and FIRST back to original form
#                     ERRORS[i] = Errors.ERROR_NONE
#                     FIRST[i]  = True
#                     TIME[i]   = 0
        
#         # Checking if the current has been under the max limit for more than 10 ms
#         elif MOTOR_CURR[i] < max_current[i] and time.time() - TIME[i] > 0.01:

#             # Check if the motor has an error associated with it already
#             if ERRORS[i] != Errors.ERROR_EXCEEDING_POS:

#                 # Remove the error and set TIME and FIRST back to original form
#                 ERRORS[i] = Errors.ERROR_NONE
#                 FIRST[i]  = True
#                 TIME[i]   = 0

# def limitSwitch_check(spark_input : list, dt : float = 0.1):
#     '''
#     (list(int)) -> (list(int))

#     Performs safety operations:
#     1) Compares the goal position with current position to see that the difference 
#     between them is not gigantic. If gigantic sets the error as for that motor as
#     Errors.ERROR_EXCEEDING_POS. Makes sure the motor doesn't jerk back to 0 
#     position in case computer loses power

#     2) Checks the maximum current being consumed by a motor. If the current is higher than
#     the expected max, sets the error for the particular motor as Errors.ERROR_EXCEEDING_CURRENT.
#     Helps us to know when too much torque is being applied

#     @parameters

#     spark_input (list(int)): Input positions for the motors
#     '''
#     pass