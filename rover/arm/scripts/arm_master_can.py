#!/usr/bin/env python3

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

# Safety Node Only
class Errors(Enum):
    ERROR_NONE = 0
    ERROR_EXCEEDING_POS = 1
    ERROR_EXCEEDING_CURRENT = 2


########## GLOBAL VARIABLES ##########

# Variable to store current position of arm motors
# CURR_POS and MOTOR_CURR are populated by CAN Node as topics 
CURR_POS = [0, 0, 0, 0, 0, 0, 0]
MOTOR_CURR = [0, 0, 0, 0, 0, 0, 0]

# ERRORS are populated by Safety Node as a topic
ERRORS = [0, 0, 0, 0, 0, 0, 0]

TIME = [0, 0, 0, 0, 0, 0, 0]
FIRST = [True, True, True, True, True, True, True]

# REDUCTION is used by CAN Node only as a global constant
REDUCTION = [120, 160, 120, 20, 20, 20, 40]

# Shared lock variable
lock = threading.Lock()

########## HELPER FUNCTIONS ##########

def read_ros_message(data):
	"""
	Reads messages from a ROS topic that are supposed to be sent to the motor controllers
	"""
	print(data.data)

def send_ros_message():
	"""
	Sends messages over a ROS topic back to IK
	"""
	pass

# CAN Node only
def generate_data_packet (data_list : list):
    """
    list(float) -> list(list(int))

    Takes in the goal position angles for each motor and converts them to bytearrays specific for
    each motor

    @parameters

    data_list (list(float)): List containing the anglular positions (in degrees) for each motor in the order
    """
    
    # A variable to keep count of joint number
    joint_num = 0
    
    # Sparkmax Data list
    spark_data = []

    for angle in data_list:
        joint_num += 1

        # Converting the angle to spark data
        angle = angle/360 * REDUCTION[joint_num - 1]
        spark_data.append(arm_can.pos_to_sparkdata(angle))

        # For debugging
        if 6 == joint_num:
            #print(angle*360/REDUCTION[5])
            # print(angle)
            # print("CURR: {}      MOTOR_CURR: {}".format(CURR_POS[5], MOTOR_CURR[5]) )
            # print(ERRORS[5])
            pass
    return spark_data

# CAN Node only
def read_message_from_spark(init : bool = False):
    """
    (None) -> (None)

    Function that runs in a separate thread that constantly reads status message regarding
    position from all the motors and updates the global variable CURR_POS to store the values
    """

    # Variable to hold the boolean whether each motor is read once or not
    # For each motor connected, the corresponding motor_list element should be set to 0 
    motor_read = [1, 1, 1, 1, 1, 0, 1]

    # Checking if SparkMAXes are powered on and sending status messages
    while 1:
        msg = arm_can.BUS.recv()
        can_id = msg.arbitration_id
        dev_id = can_id & 0b00000000000000000000000111111
        api = (can_id >> 6) & 0b00000000000001111111111
        
        # If every element in motor_list is True, it means this was for initialization 
        # and we should break out of the loop
        if not False in motor_read:
            break

        index = dev_id - 11

        if api == arm_can.CMD_API_STAT1:
            # Starting thread lock
            with lock:
                MOTOR_CURR[index] = arm_can.read_can_message(msg.data, api)
            # Ending thread lock
        
        elif api == arm_can.CMD_API_STAT2:

            # If this is for initialization, set the correseponding element in motor_list to be True
            if init:
                motor_read[index] = True

            # Starting thread lock
            with lock:
                CURR_POS[index] = arm_can.read_can_message(msg.data, api)
            # Ending thread lock    

# Safety Node only             
def safety(spark_input : list, dt : float = 0.1):
    '''
    (list(int)) -> (list(int))

    Performs safety operations:
    1) Compares the goal position with current position to see that the difference 
    between them is not gigantic. If gigantic sets the error as for that motor as
    Errors.ERROR_EXCEEDING_POS. Makes sure the motor doesn't jerk back to 0 
    position in case computer loses power

    2) Checks the maximum current being consumed by a motor. If the current is higher than
    the expected max, sets the error for the particular motor as Errors.ERROR_EXCEEDING_CURRENT.
    Helps us to know when too much torque is being applied

    @parameters

    spark_input (list(int)): Input positions for the motors
    '''

    # Multiplying factors for position safety
    factor = [0.6, 0.8, 0.15, 0.05, 1/15, 1/15, 1/50]

    # Checking if input is as long as CURR_POS
    if len(spark_input) == len(CURR_POS):

        for i in range(len(spark_input)):
            
            # Getting the float value from potential input
            float_val = arm_can.read_can_message(spark_input[i], arm_can.CMD_API_STAT2)
            
            # Starting thread lock
            with lock:
                #print(dt)
                # Doing position comparisons for safety
                if float_val < (CURR_POS[i] - factor[i] * speed_limit[i] * 0.1) or float_val > (CURR_POS[i] + factor[i] * speed_limit[i] * 0.1):

                    # Calculating the offset and applying it to controller input
                    offset = float_val - CURR_POS[i]
                    controller_pos[i] -= offset * 360 / REDUCTION[i]
                    #spark_input[i] = arm_can.pos_to_sparkdata(CURR_POS[i])

                    # Set the error for the motor
                    ERRORS[i] = Errors.ERROR_EXCEEDING_POS

                    # For debugging safety code, you can uncomment this
                    #spark_input[i] = arm_can.pos_to_sparkdata(CURR_POS[i])

                # Remove the error if it is no longer there
                # By design, the EXCEEDING_POS error should not be there after one 
                # cycle of the main loop as the offset to controller input will bring
                # the input back to a safe value.
                else:
                    if ERRORS[i] != Errors.ERROR_EXCEEDING_CURRENT:
                        ERRORS[i] = Errors.ERROR_NONE

                # Checking maximum currents
                if MOTOR_CURR[i] > 14:

                    # Checking if the input from controller is making it go further into 
                    # the direction that increases current
                    if (abs(float_val) >= abs(CURR_POS[i])):
                        if (arm_can.sign(float_val) == arm_can.sign(CURR_POS[i])):
                            #spark_input[i] = arm_can.pos_to_sparkdata(CURR_POS[i])

                            # Check if the motor has an associated with it already
                            if ERRORS[i] != Errors.ERROR_EXCEEDING_POS:

                                # Change the error associated with the motor to EXCEEDING_CURRENT
                                ERRORS[i] = Errors.ERROR_EXCEEDING_CURRENT

                                # Check if this is the first time this error is set 
                                if FIRST[i]:

                                    # Collect time stamp since the error was set
                                    TIME[i] = time.time()
                                    FIRST[i] = False
                    
                    # If the controller input is in the direction that reduces the current
                    else:
                        if (arm_can.sign(float_val) == arm_can.sign(CURR_POS[i])):

                            # Check if the motor has an associated with it already
                            if ERRORS[i] != Errors.ERROR_EXCEEDING_POS:

                                # Remove the error and set TIME and FIRST back to original form
                                ERRORS[i] = Errors.ERROR_NONE
                                FIRST[i] = True
                                TIME[i] = 0
                
                # Checking if the current has been under the max limit for more than 10 ms
                elif MOTOR_CURR[i] < 14 and time.time() - TIME[i] > 0.01:

                    # Check if the motor has an associated with it already
                    if ERRORS[i] != Errors.ERROR_EXCEEDING_POS:

                        # Remove the error and set TIME and FIRST back to original form
                        ERRORS[i] = Errors.ERROR_NONE
                        FIRST[i] = True
                        TIME[i] = 0

        #print("Input:", arm_can.read_can_message(spark_input[4], arm_can.CMD_API_STAT2))
        return spark_input
        
    else:
        print('ERROR: "spark_input" is invalid, returning CURR_POS')
        return CURR_POS

                



############################## MAIN ##############################

if __name__=="__main__":

    # CAN Node only
    # Instantiating the CAN bus
    arm_can.initialize_bus()

    # CAN Node only
    # Creating the message packet for Heartbeat
    hb = arm_can.can.Message(
        arbitration_id= arm_can.generate_can_id(
            dev_id= 0x0, 
            api= arm_can.CMD_API_NONRIO_HB), 
        data= bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]), 
        is_extended_id= True,
        is_remote_frame = False, 
        is_error_frame = False
    )

    # CAN Node only
    # Broadcasting the heartbeat
    task = arm_can.BUS.send_periodic(hb, 0.01)
    print("Heartbeat initiated")

    # Initializing CURR_POS variable
    read_message_from_spark(init= True)

    # Starting a thread to read current position of motors
    curr_pos_thread = threading.Thread(target= read_message_from_spark, args= (), daemon= True)
    curr_pos_thread.start()

    # Variable Declaration for input from motor controller
    input_angles = []

    # rospy.init_node("arm_CAN")
    # rospy.Subscriber("ik_angles", Float32MultiArray, read_ros_message)

    # Variable to hold current configuration of servo, starting with 63 degrees always
    triggered = 0

    # Starting the infinite loop
    while 1:
        
        # Getting angles from the remote controller
        # Safety Node Only
        input_angles = MapJoystick(GetManualJoystickFinal.GetManualJoystick(), controller_pos, speed_limit, t-time.time())
        t = time.time()

        # Printing input angles from remote controller for debugging
        #print(input_angles)

        # Converting received SparkMAX angles to SparkMAX data packets
        # CAN Node only
        spark_input = generate_data_packet(input_angles[:7])
        # Comparison between spark_input and CURR_POS for safety
        # Safety Node only
        spark_input = safety(spark_input, time.time()-GetManualJoystickFinal.time_event)
        # if MOTOR_CURR > 14:
        #     print("motor current:", MOTOR_CURR)
        #     print("Exceeding Max Current")

        #     for i in range(len(spark_input)):
        #         spark_input[i] = arm_can.pos_to_sparkdata(CURR_POS[i])
        #     continue

        # Move the motors only if there are no errors in any of them
        if ERRORS.count(Errors.ERROR_NONE) == len(ERRORS): # Safety Node only

            # CAN Node only
            # Sending data packets one by one
            for i in range(1, len(spark_input)+1):
                
                # Motor number corresponds with device ID of the SparkMAX
                motor_num = 10 + i

                if motor_num > 10 and motor_num < 18:
                    id = arm_can.generate_can_id(dev_id= motor_num, api= arm_can.CMD_API_POS_SET)
                    arm_can.send_can_message(can_id= id, data= spark_input[i - 1])
                
                else:
                    break
            
        # Toggling End Effector configuration using servo
        
        # Manual/Controller Node only
        # Going 63 degrees configuration if not in this configuration
        if input_angles[7] == 0 and triggered != 0: 
            triggered = 0
            #servo.write_servo_low_angle()
            print("Set low")
        
        # Going 84 degrees configuration if not in this configuration
        elif input_angles[7] == 1 and triggered != 1:
            triggered = 1
            #servo.write_servo_high_angle()
            print("Set high")
        #time.sleep(.2)
        #print("loop time, MOT_CURR:", time.time() - ti, MOTOR_CURR[5])


    # rospy.spin()