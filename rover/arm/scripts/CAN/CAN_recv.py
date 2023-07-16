#!/usr/bin/env python3

#from CAN_utilities import *
import rospy
from std_msgs.msg import Float32MultiArray, UInt8MultiArray
import can
import struct

########## GLOBAL VARIABLES ##########

# CAN bus instance
global BUS

# CANSpark APIs
CMD_API_SETPNT_SET = 0x001
CMD_API_DC_SET = 0x002
CMD_API_SPD_SET = 0x012
CMD_API_SMART_VEL_SET = 0x013
CMD_API_POS_SET = 0x032
CMD_API_VOLT_SET = 0x042 
CMD_API_CURRENT_SET = 0x043
CMD_API_SMARTMOTION_SET = 0x052
CMD_API_STAT0 = 0x060
CMD_API_STAT1 = 0x061
CMD_API_STAT2 = 0x062
CMD_API_STAT3 = 0x063
CMD_API_STAT4 = 0x064
CMD_API_STAT5 = 0x065
CMD_API_STAT6 = 0x066
CMD_API_CLEAR_FAULTS = 0x06E
CMD_API_DRV_STAT = 0x06A
CMD_API_BURN_FLASH = 0x072
CMD_API_SET_FOLLOWER = 0x073
CMD_API_FACTORY_DEFAULT = 0x074
CMD_API_FACTORY_RESET = 0x075
CMD_API_IDENTIFY = 0x076
CMD_API_NACK = 0x080
CMD_API_ACK = 0x081
CMD_API_BROADCAST = 0x090
CMD_API_HEARTBEAT = 0x092
CMD_API_SYNC = 0x093
CMD_API_ID_QUERY = 0x094
CMD_API_ID_ASSIGN = 0x095
CMD_API_FIRMWARE = 0x098
CMD_API_ENUM = 0x099
CMD_API_LOCK = 0x09B
CMD_API_LOCKB = 0x0B1
CMD_API_NONRIO_HB = 0x0B2

CMD_API_SWDL_BOOTLOADER = 0x1FF
CMD_API_SWDL_DATA = 0x09C
CMD_API_SWDL_CHKSUM = 0x09D
CMD_API_SWDL_RETRANSMIT = 0x09E

CMD_API_MECH_POS = 0x0A0
CMD_API_I_ACCUM = 0x0A2
CMD_API_ANALOG_POS = 0x0A3
CMD_API_ALT_ENC_POS = 0x0A4
CMD_API_PARAM_ACCESS = 0x300

# REDUCTION is used by CAN Node only as a global constant
REDUCTION = [120, 160, 120, 20, 20, 20, 40]

########## SHARED FUNCTIONS ##########

def generate_can_id(dev_id : int, api : int, 
						man_id = 0x05, dev_type = 0x2):
		"""
		(int, int, int, int, int) -> (int)

		Generates a valid extended CAN ID for a Rev object (SparkMAX by default) 
		with id "dev_id" and API call "api"

		@parameters

		dev_id (int) = The unique devide ID of the object in the CAN network (6 bits)
		api (int) = The API for SparkMAX that is required to be run (10 bits)
		man_id (int) (optional) = Manufactured ID, default value is 5 (8 bits)
		dev_type (int) (optional) = Device Type, default value is 2 (5 bits)

		Note: The complete CAN ID should never be larger than 29 bits
		"""

		# Forming the CAN ID
		can_id = 0 << 5
		can_id = (can_id | dev_type) << 8
		can_id = (can_id | man_id) << 10
		can_id = (can_id | api) << 6
		can_id = (can_id | dev_id)
		return can_id


def pos_to_sparkdata(f : float):
    """
    float -> list()

    Takes in a float position value (number of rotations) and returns the data packet in the form that
    SparkMAX requires

    @parameters:

    f (float): Float value that needs to be converted
    """
    input_hex =  hex(struct.unpack('<I', struct.pack('<f', f))[0])
    if len(input_hex) != 10:
        input_hex = input_hex[:2] + input_hex[2:] + (10-len(input_hex))*'0'
    
    return [eval('0x'+input_hex[-2:]), eval('0x'+input_hex[-4:-2]),
            eval('0x'+input_hex[-6:-4]), eval('0x'+input_hex[-8:-6]),
            0x00, 0x00, 0x00, 0x00]


def sparkfixed_to_float(fixed : int, frac : int = 5):
    """
    Returns floating point representation of the fixed point represenation of 
    data received from SparkMAX

    @parameters:
    fixed (int): Input fixed point number from SparkMAX
    frac (int) (optional): Number of fractional bits
    """
    # Divide the received value by 2^(number of fractional bits)
    f = fixed / (2 ** frac)
    return f



def initialize_bus(channel= 'can0', interface= 'socketcan'):
    """
    (str, str) -> (void)

    Creates an instance of the CAN bus for sending and receiving
    CAN messages. By defaul, the network it searches for is named 'can0'
    and the interfaces it uses is 'socketcan'. For a detailed list of 
    available interfaces, check out the python-can documentation

    @parameters

    channel (str) (optional) = Name of the CAN network you are using
    interface (str) (optional) = Name of the interface you are using

    """
    # Getting global BUS
    global BUS

    # Initializing the global BUS
    BUS = can.ThreadSafeBus(channel= channel, interface= interface, receive_own_messages= False)
    print('BUS initialzed')
    return


def send_can_message(can_id : int, data = None, ext = True, err = False, rtr = False):
    """
    (int, list(float), bool, bool, bool)

    Forms and sends the complete CAN packet with the given data and can_id

    @parameters

    can_id (int) = The complete 29 bit CAN address, can be generated from generate_can_id
    data (list(float)) = An iterable object of ints or bytes, data can be max 64 bit
    ext (bool) (optional) = True if can_id is extended 29 bits, False if it is 11 bits.
        True by default
    err (bool) (optional) = True if it is an error frame, False otherwise.
        False by default
    rtr (bool) (optional) = True if it is remote frame, False otherwise.
        False by default
    """

    # Getting global BUS
    global BUS

    # Converting list data to byte
    if data:
        data = bytes(data)

    # Creating the message packet
    msg = can.Message(
        arbitration_id= can_id,
        data= data,
        is_extended_id= ext,
        is_remote_frame = rtr,
        is_error_frame = err
    )

    # Sending the created message
    try:
        BUS.send(msg)
        #print(f"Message sent on {BUS.channel_info}")

    except can.CanError:
        #print("Message NOT sent")
        pass
    return


def read_can_message(data, api):
    """
    Converts CAN data packets from hex to float decimal values based on which API is 
    called

    *******NEEDS TO BE COMPLETED*******
    @parameters

    data (bytearray) = The CAN data packet containing just the payload. 
    api (int) = The API you want the payload of.
    """
    
    if api:
        # API: Status Message 1 - Gives us information on limit switches
        if api == CMD_API_STAT0:
            # 132 for forward and 68 for reverse
            ls_bools = data[3] & 0xC0

            # Return 0 if no limit switches are pressed
            if ls_bools > 0:
                return 0
            # Return 1 if either forward or reverse limit switches are pressed
            else:
                return 1

        # API: Status Message 1 - Gives us motor velocity, motor voltage and
        # motor current every 20ms. We only need current
        elif api == CMD_API_STAT1:

            # Getting the motor current value stored as hexadecimals 
            currVal_fixed = (data[-1] << 4) | ((data[-2] & 0xF0) >> 4)

            # SparkMAX stores motor current value in fixed point format,
            # need to convert it to float for reading
            currVal_float = sparkfixed_to_float(currVal_fixed)

            return currVal_float
        
        # API: Status Message 2 - Gives us current position and comes every 20ms
        elif api == CMD_API_STAT2:
            
            # Checking if all the bits are 0 or not, if yes return 0
            if not(data[3] or data[2] or data[1] or data[0]):
                return 0

            # Extracting the position value bytes
            pos_float = data[3] << 8
            pos_float = (pos_float | data[2]) << 8
            pos_float = (pos_float | data[1]) << 8
            pos_float = (pos_float | data[0])

            # Converting the extracted bytes to hex
            pos_hex = str(hex(pos_float))

            # Check if we have 8 hex characters in pos_hex, if not then pad it with zeros
            if len(pos_hex) != 10:
                pos_hex = format(pos_float, '#010x')
            
            # Converting the hex representation to floating point decimal value
            pos_float = struct.unpack('!f', bytes.fromhex(pos_hex[2:]))[0]

            return pos_float

    else:
        # Error Message, invalid API
        print("Invalid API ID")
        return -1


# NEEDS CALIBRATION
def calc_differential(d_pos, d_angle):
    """
    (float), (float) --> list(float). In same units as safe_goal_pos

    Given how much you want the arm(gear?)(joint?) to rotate about it's axis (d_angle) and
    rotate as a big lever (d_pos), returns how much each of the two motors need to rotate

    DIRECTION NEEDS TO BE CALIBRATED BASED ON MOTOR POSITIONS AND REFERENCE COORDINATES
    """


    # 2:1 gear ratio (2 turns of small gear = 1 turn of big gear) (I think)
    # Motor movement required to produce d_angle
    gear_ratio = 2.0/1.0

    d_angle_motor1 = d_angle * gear_ratio # assuming same gear ratios
    d_angle_motor2 = -d_angle * gear_ratio

    # correction through the gripper motor to stop the gripper from opening and closing
    d_gripper_motor = -d_angle # rotate the nut in the opposite direction same amount (no gear ratios)


    # Motor movement required to produce d_pos
    # Moves the entire joint so gear ratio doesn't matter
    d_angle_motor1 += d_pos
    d_angle_motor2 += d_pos

    
    return d_angle_motor1, d_angle_motor2, d_gripper_motor


def generate_data_packet(data_list : list):
    """
    list(float) -> list(list(int))

    Takes in the goal position angles for each motor and converts them to bytearrays specific for
    each motor

    @parameters

    data_list (list(float)): List containing the anglular positions (in degrees) for each motor in the order
    """
    
    # Angle conversion for differential system
    # Assuming the last two angles specify the angle of the differential system,
    # convert those two values to the required angles for motors 5 and 6
    data_list[-3], data_list[-2], correction = calc_differential(data_list[len(data_list)-2], data_list[len(data_list)-1])
    data_list[-1] += correction # correction for roll

    # A variable to keep count of joint number
    joint_num = 0
    
    # Sparkmax Data list
    spark_data = []

    for angle in data_list:
        joint_num += 1

        # Converting the angle to spark data
        angle = angle/360 * REDUCTION[joint_num - 1]
        spark_data.append(pos_to_sparkdata(angle))

    return spark_data

class CAN_Recv_Node():
    """
    (None)

    This class represents an instance of the CAN node and connects the ROS node
    to multiple topics by making it a subscriber or a publisher
    """

    def __init__(self):
        self.pub_rate           = 100000000

        # Attributes to hold data from publishers or to publish
        self.CURR_POS 			= [0, 0, 0, 0, 0, 0, 0]
        self.MOTOR_CURR 		= [0, 0, 0, 0, 0, 0, 0]
        self.LIMIT_SWITCH 		= [0, 0, 0, 0, 0, 0, 0]

        # Variables for ROS publishers and subscribers
        self.Angles_pub 		= rospy.Publisher('arm_curr_pos', Float32MultiArray, queue_size=10)
        self.Motor_pub 			= rospy.Publisher('arm_motor_curr', Float32MultiArray, queue_size=10)
        self.LMS_pub 			= rospy.Publisher('arm_limit_switch', UInt8MultiArray, queue_size=10)

        # ROS
        try:
            self.CAN_recv_node()
        except rospy.ROSInterruptException:
            print("Exception Occured")
            pass


    def read_message_from_spark(self, init= False):
        """
        (None) -> (None)

        Function that reads status message regarding position and current 
        from all motors and updates the global variable CURR_POS and MOTOR_CURR to store the values
        """

        # Variable to hold the boolean whether each motor is read once or not
        # For each motor connected, the corresponding motor_list element should be set to 0 
        #motor_read = [1, 1, 1, 1, 1, 0, 1]

        # Checking if SparkMAXes are powered on and sending status messages
        while 1:
            msg     = BUS.recv()
            can_id  = msg.arbitration_id
            dev_id  = can_id & 0b00000000000000000000000111111
            api     = (can_id >> 6) & 0b00000000000001111111111
            
            # If every element in motor_list is True, it means this was for initialization 
            # and we should break out of the loop
            # if not False in motor_read:
            # 	break
            
            index   = dev_id - 11

            # If this is for initialization, set the correseponding element in motor_list to be True
            # if init:
            # 	motor_read[index] = True
            if api == CMD_API_STAT0:
                self.LIMIT_SWITCH[index] = read_can_message(msg.data, CMD_API_STAT0)
            elif api == CMD_API_STAT1:
                self.MOTOR_CURR[index]   = read_can_message(msg.data, CMD_API_STAT1)
            elif api == CMD_API_STAT2:
                self.CURR_POS[index]     = read_can_message(msg.data, CMD_API_STAT2)


    def CAN_recv_node(self): 	# Queue size & rate not calibrated
        # # Instantiate CAN bus
        # initialize_bus()

        # # Broadcast heartbeat
        # hb = can.Message(
        #     arbitration_id= generate_can_id(
        #         dev_id= 0x0, 
        #         api= CMD_API_NONRIO_HB), 
        #     data= bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]), 
        #     is_extended_id= True,
        #     is_remote_frame = False, 
        #     is_error_frame = False
        # )
        # task = BUS.send_periodic(hb, 0.01)
        # print("Heartbeat initiated")
        
        # Set publishing rate to 10hz
        rate = rospy.Rate(self.pub_rate)

        while not rospy.is_shutdown():
            # Parse updated motor angles and current
            msg = BUS.recv()
            can_id = msg.arbitration_id
            dev_id = can_id & 0b00000000000000000000000111111
            api = (can_id >> 6) & 0b00000000000001111111111
            
            index = dev_id - 11

            if api == CMD_API_STAT0:
                self.LIMIT_SWITCH[index] = read_can_message(msg.data, CMD_API_STAT0)

                # Publish data
                limit_switch_msg = UInt8MultiArray()
                limit_switch_msg.data = self.LIMIT_SWITCH
                self.LMS_pub.publish(limit_switch_msg)

            elif api == CMD_API_STAT1:
                self.MOTOR_CURR[index] = read_can_message(msg.data, CMD_API_STAT1)

                # Publish data
                MOTOR_CURR_msg = Float32MultiArray()
                MOTOR_CURR_msg.data = self.MOTOR_CURR
                self.Motor_pub.publish(MOTOR_CURR_msg)
                
            elif api == CMD_API_STAT2:
                self.CURR_POS[index] = read_can_message(msg.data, CMD_API_STAT2)

                # Publish data
                curr_angles_msg = Float32MultiArray() 
                curr_angles_msg.data = self.CURR_POS
                self.Angles_pub.publish(curr_angles_msg)


            # Control rate
            rate.sleep()


if __name__=="__main__":

    # Instantiate CAN bus
    initialize_bus()
    
    # Broadcast heartbeat
    hb = can.Message(
        arbitration_id= generate_can_id(
            dev_id= 0x0, 
            api= CMD_API_NONRIO_HB), 
        data= bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]), 
        is_extended_id= True,
        is_remote_frame = False, 
        is_error_frame = False
    )
    task = BUS.send_periodic(hb, 0.01)
    print("Heartbeat initiated")

    # Initialize node
    rospy.init_node('CAN_Recv', anonymous=True)

    # Setup and run node
    CAN = CAN_Recv_Node()

    # Spin to keep node alive
    rospy.spin() # Not sure this is actually needed since infinite loop is used to parse CAN messages
