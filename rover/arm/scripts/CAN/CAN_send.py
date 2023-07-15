from CAN_utilities import *


class CAN_Send_Node():
	"""
	(None)

	This class represents an instance of the CAN node and connects the ROS node
	to multiple topics by making it a subscriber or a publisher
	"""

	def __init__(self):
		self.pub_rate = 0

		# Subscriber
		self.SAFE_GOAL_POS	 	= [0, 0, 0, 0, 0, 0, 0]

		# Variables for ROS publishers and subscribers
		self.SafePos_sub 		= rospy.Subscriber("arm_safe_goal_pos", Float32MultiArray, self.callback_SafePos)

		# ROS
		try:
			print("here")
			self.CAN_send_node()
		except rospy.ROSInterruptException:
			print("Exception Occured")
			pass
		

	def callback_SafePos(self, data):
		self.SAFE_GOAL_POS = data.data

	
	def CAN_send_node(self): 	# Queue size & rate not calibrated
		# # Instantiate CAN bus
		# initialize_bus()

		# # Broadcast heartbeat
		# hb = can.Message(
		# 	arbitration_id= generate_can_id(
		# 		dev_id= 0x0, 
		# 		api= CMD_API_NONRIO_HB), 
		# 	data= bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]), 
		# 	is_extended_id= True,
		# 	is_remote_frame = False, 
		# 	is_error_frame = False
		# )
		# task = BUS.send_periodic(hb, 0.01)
		# print("Heartbeat initiated")
		
		# Set publishing rate to 10hz
		rate = rospy.Rate(self.pub_rate)

		while not rospy.is_shutdown():
			# Send instructions to motor
			# Convert SparkMAX angles to SparkMAX data packets
			spark_input = generate_data_packet(self.SAFE_GOAL_POS) # assuming data is safe
			
			# Send data packets
			for i in range(1, len(spark_input)+1):
						
				# Motor number corresponds with device ID of the SparkMAX
				motor_num = 10 + i

				if motor_num > 10 and motor_num < 18:
					id = generate_can_id(dev_id= motor_num, api= CMD_API_POS_SET)
					send_can_message(can_id= id, data= spark_input[i - 1])
				
				else:
					break

			# Control rate
			rate.sleep()


if __name__=="__main__":
	# Initialize node
	rospy.init_node('CAN_Send', anonymous=True)

	# Setup and run node
	CAN = CAN_Send_Node()

	# Spin to keep node alive
	rospy.spin() # Not sure this is actually needed since infinite loop is used to parse CAN messages
