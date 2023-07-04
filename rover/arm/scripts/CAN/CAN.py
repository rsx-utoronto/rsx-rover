#!/usr/bin/env python3
import can
import rospy
import time
import struct
from enum import Enum
from std_msgs.msg import Float32MultiArray

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


# Variable to store current position of arm motors
# CURR_POS and MOTOR_CURT are populated by CAN Node as topics 
CURR_POS = [0, 0, 0, 0, 0, 0, 0]
MOTOR_CURT = [0, 0, 0, 0, 0, 0, 0]

SAFE_GOAL_POS = [0, 0, 0, 0, 0, 0, 0]

# REDUCTION is used by CAN Node only as a global constant
REDUCTION = [120, 160, 120, 20, 20, 20, 40]

########## CLASSES ##########

########## HELPER FUNCTIONS ##########

# CAN Node only
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

# CAN Node only
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

# CAN Node only
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


# CAN Node only
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

# CAN Node only
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

# CAN Node only
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
		# API: Status Message 1 - Gives us motor velocity, motor voltage and
		# motor current every 20ms. We only need current
		if api == CMD_API_STAT1:

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
        spark_data.append(pos_to_sparkdata(angle))

        # For debugging
        if 6 == joint_num:
            #print(angle*360/REDUCTION[5])
            # print(angle)
            # print("CURR: {}      MOTOR_CURT: {}".format(CURR_POS[5], MOTOR_CURT[5]) )
            # print(ERRORS[5])
            pass
    return spark_data


# CAN Node only
def read_message_from_spark(init= False):
	"""
	(None) -> (None)

	Function that reads status message regarding position and current 
	from all motors and updates the global variable CURR_POS and MOTOR_CURT to store the values
	"""

	# Variable to hold the boolean whether each motor is read once or not
	# For each motor connected, the corresponding motor_list element should be set to 0 
	motor_read = [1, 1, 1, 1, 1, 0, 1]

	# Checking if SparkMAXes are powered on and sending status messages
	while 1:
		msg = BUS.recv()
		can_id = msg.arbitration_id
		dev_id = can_id & 0b00000000000000000000000111111
		#api = (can_id >> 6) & 0b00000000000001111111111
		
		# If every element in motor_list is True, it means this was for initialization 
		# and we should break out of the loop
		if not False in motor_read:
			break
		
		index = dev_id - 11

		# If this is for initialization, set the correseponding element in motor_list to be True
		if init:
			motor_read[index] = True
		
		MOTOR_CURT[index] = read_can_message(msg.data, CMD_API_STAT1)
		CURR_POS[index] = read_can_message(msg.data, CMD_API_STAT2)


def callback(data):
	global SAFE_GOAL_POS
	SAFE_GOAL_POS = data.data


def CAN_node(): 	# Queue size & rate not calibrated
	# Setup ROS node
	# Publisher to angle position and motor current topics
	angles_pub = rospy.Publisher('curr_angles', Float32MultiArray, queue_size=10)
	motor_pub = rospy.Publisher('motor_curt', Float32MultiArray, queue_size=10)

	# Create node
	rospy.init_node('CAN', anonymous=True)

	# Set publishing rate to 10hz
	rate = rospy.Rate(10)

	# Subscribe to safe goal position topic
	safe_goal_sub = rospy.Subscriber('safe_goal_pos', Float32MultiArray, callback) # should callback whenever there is new message
	
	while not rospy.is_shutdown():
		# Send instructions to motor
		# Convert SparkMAX angles to sparkMAX data packets
		spark_input = generate_data_packet(SAFE_GOAL_POS) # assuming data is safe
		
		# Send data packets
		for i in range(1, len(spark_input)+1):
					
			# Motor number corresponds with device ID of the SparkMAX
			motor_num = 10 + i

			if motor_num > 10 and motor_num < 18:
				id = generate_can_id(dev_id= motor_num, api= CMD_API_POS_SET)
				send_can_message(can_id= id, data= spark_input[i - 1])
			
			else:
				break

		# Parse updated motor angles and current
		read_message_from_spark()
		
		# Create ROS messages and intialize value
		curr_angles_msg = Float32MultiArray()
		motor_curt_msg = Float32MultiArray()

		curr_angles_msg.data = CURR_POS
		motor_curt_msg.data = MOTOR_CURT

		# Publish current angles and motor current
		angles_pub.publish(curr_angles_msg)
		motor_pub.publish(motor_curt_msg)

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

	# Initialize CURR_POS variable
	read_message_from_spark(init= True)

	# ROS
	try:
		CAN_node()
	except rospy.ROSInterruptException:
		print("Exception Occured")
		pass
