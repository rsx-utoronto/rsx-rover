#!/usr/bin/env python3

import arm_can_control as arm_can
from GetManualJoystickFinal import *
from MapManualJoystick import *
#import arm_servo as servo
import struct
import rospy
from std_msgs.msg import Float32MultiArray
import threading

########## GLOBAL VARIABLES ##########

# Variable to store current position of arm motors
CURR_POS = [0, 0, 0, 0, 0, 0, 0]

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

def generate_data_packet (data_list : list):
    """
    list(float) -> list(list(int))

    Takes in the goal position angles for each motor and converts them to bytearrays specific for
    each motor

    @parameters

    data_list (list(float)): List containing the anglular positions for each motor in the order
    """
    
    # A variable to keep count of joint number
    joint_num = 0
    
    # Sparkmax Data list
    spark_data = []

    for angle in data_list:
        joint_num += 1
        
        # Gear reduction
        if joint_num <= 4: 
            reduction = 100
        else:
            reduction = 20

        # Checking which joint to move
        if joint_num == 1:
            angle = angle/360 * reduction
            spark_data.append(arm_can.pos_to_sparkdata(angle))
        
        elif joint_num == 2:
            angle = angle/360 * reduction
            spark_data.append(arm_can.pos_to_sparkdata(angle))

        elif joint_num == 3:
            angle = angle/360 * reduction
            spark_data.append(arm_can.pos_to_sparkdata(angle))
        
        elif joint_num == 4:
            angle = angle/360 * reduction
            spark_data.append(arm_can.pos_to_sparkdata(angle))
        
        elif joint_num == 5:
            angle = angle/360 * reduction
            spark_data.append(arm_can.pos_to_sparkdata(angle))

        elif joint_num == 6:
            angle = angle/360 * reduction
            spark_data.append(arm_can.pos_to_sparkdata(angle))

        elif joint_num == 7:
            angle = angle/360 * reduction
            spark_data.append(arm_can.pos_to_sparkdata(angle))

        else:
            print("Error: Reaching infinite loop/Out of Index")
            break
    
    return spark_data

def read_pos_from_spark():
    """
    (None) -> (None)

    Function that runs in a separate thread that constantly reads status message regarding
    position from all the motors and updates the global variable CURR_POS to store the values
    """

    # Checking if SparkMAXes are powered on and sending status messages
    while 1:
        msg = arm_can.BUS.recv()
        can_id = msg.arbitration_id
        dev_id = can_id & 0b00000000000000000000000111111
        api = (can_id >> 6) & 0b00000000000001111111111

        if api != arm_can.CMD_API_STAT2:
            # Skip to the next iteration of the loop
            continue
        
        else:
            # Starting thread lock
            with lock:
                index = dev_id - 11
                CURR_POS[index] = arm_can.read_can_message(msg.data, api)
            # Ending thread lock
    
def cmp_goal_curr_pos(spark_input : list):
    '''
    (list(int)) -> (list(int))

    Compares the goal position with current position to see that the difference 
    between them is not gigantic for safety. Updates the goal position if unsafe

    @parameters

    spark_input (list(int)): Input positions for the motors
    '''

    # Starting thread lock
    with lock:

        # Checking if input is as long as CURR_POS
        if len(spark_input) == len(CURR_POS):

            for i in range(len(spark_input)):

                # Doing comparisons for safety
                if spark_input[i] < (CURR_POS[i] - dt * speed) or spark_input[i] > (CURR_POS[i] + dt * speed):
                    spark_input[i] = CURR_POS[i]
            return spark_input
        
        else:
            print('ERROR: "spark_input" is invalid, returning CURR_POS')
            return CURR_POS

                



############################## MAIN ##############################

if __name__=="__main__":
    
    # Instantiating the CAN bus
    arm_can.initialize_bus()

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

    # Broadcasting the heartbeat
    task = arm_can.BUS.send_periodic(hb, 0.01)
    print("Heartbeat initiated")

    # Starting a thread to read current position of motors
    curr_pos_thread = threading.Thread(target= read_pos_from_spark, args= ())
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
        input_angles = MapJoystick(GetManualJoystickFinal.GetManualJoystick(), current_pos, speed_limit, t-time.time())
        t = time.time()

        # Printing input angles from remote controller
        print(input_angles)
        
        # Converting received SparkMAX angles to SparkMAX data packets
        spark_input = generate_data_packet(input_angles[:7])

        # Comparison between spark_input and CURR_POS for safety
        ################# TO DO ###################
        # cmp_goal_curr_pos(spark_input)

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


# rospy.spin()