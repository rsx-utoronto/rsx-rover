from CAN_utilities import *
import can

# Instantiate CAN bus
BUS = initialize_bus()

def read_message_from_spark(msg : can.Message):
    """
    (can.Message) -> (None)

    Function that reads status message regarding position, limit switch and current 
    from all motors and updates the global variable CURR_POS, LIMIT_SWITCH and MOTOR_CURR
    to store the values
    """

    # Variable to hold the boolean whether each motor is read once or not
    # For each motor connected, the corresponding motor_list element should be set to 0 
    #motor_read = [1, 1, 1, 1, 1, 0, 1]

    # Checking if SparkMAXes are powered on and sending status messages
    can_id  = msg.arbitration_id
    dev_id  = can_id & 0b00000000000000000000000111111
    api     = (can_id >> 6) & 0b00000000000001111111111
    #man_id  = (can_id) & 0b00000111111110000000000000000
    
    # If every element in motor_list is True, it means this was for initialization 
    # and we should break out of the loop
    # if not False in motor_read:
    # 	break
    
    # Getting the list index value based on motor device id
    index   = dev_id - 11

    # If this is for initialization, set the correseponding element in motor_list to be True
    # if init:
    # 	motor_read[index] = True
    #print(dev_id)
    if dev_id >  10:
        # # API for reading limit switch
        # if api == CMD_API_STAT0:

        #     # Update the LIMIT_SWITCH data
        #     print("Limit Switch Message Received")
        #     print(api)
        #     print(can_id)
        #     print("\n")
        # API for reading motor current
        if api == CMD_API_STAT1:

            # Getting the motor current value stored as hexadecimals 
            currVal_fixed = (msg.data[-1] << 4) | ((msg.data[-2] & 0xF0) >> 4)

            # SparkMAX stores motor current value in fixed point format,
            # need to convert it to float for reading
            currVal_float = sparkfixed_to_float(currVal_fixed)
            # Update the MOTOR_CURR data
            print(read_can_message(msg.data, CMD_API_STAT1))
            print(hex(api))
            print(hex(can_id))
            print("\n")

        # # API for reading current position of motor
        # elif api == CMD_API_STAT2:

        #     # Update the CURR_POS data
        #     print(read_can_message(msg.data, CMD_API_STAT2, index))
        #     print(hex(api))
        #     print(hex(can_id))
        #     print("\n")

            # Check if we updated wrist motors and apply the conversions
            # if index == 4 or index == 5:
            #     wrist1_angle       = self.CURR_POS[4]
            #     wrist2_angle       = self.CURR_POS[5]
            #     self.CURR_ANGLE[4] = (wrist1_angle + wrist2_angle) / (2 * WRIST_RATIO)
            #     self.CURR_ANGLE[5] = (wrist1_angle - wrist2_angle) / 2
            
            # else:
            #     self.CURR_ANGLE[index] = self.CURR_POS[index]

while True:
    msg = BUS.recv(timeout= 1)
    read_message_from_spark(msg)
