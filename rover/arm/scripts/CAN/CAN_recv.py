from CAN_utilities import *


class CAN_Recv_Node():
    """
    (None)

    This class represents an instance of the CAN node and connects the ROS node
    to multiple topics by making it a subscriber or a publisher
    """

    def __init__(self):
        self.pub_rate = 0

        # Attributes to hold data from publishers or to publish
        self.CURR_POS 			= [0, 0, 0, 0, 0, 0, 0]
        self.MOTOR_CURT 		= [0, 0, 0, 0, 0, 0, 0]
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
        from all motors and updates the global variable CURR_POS and MOTOR_CURT to store the values
        """

        # Variable to hold the boolean whether each motor is read once or not
        # For each motor connected, the corresponding motor_list element should be set to 0 
        #motor_read = [1, 1, 1, 1, 1, 0, 1]

        # Checking if SparkMAXes are powered on and sending status messages
        while 1:
            msg = BUS.recv()
            can_id = msg.arbitration_id
            dev_id = can_id & 0b00000000000000000000000111111
            api = (can_id >> 6) & 0b00000000000001111111111
            
            # If every element in motor_list is True, it means this was for initialization 
            # and we should break out of the loop
            # if not False in motor_read:
            # 	break
            
            index = dev_id - 11

            # If this is for initialization, set the correseponding element in motor_list to be True
            # if init:
            # 	motor_read[index] = True
            if api == CMD_API_STAT0:
                self.LIMIT_SWITCH[index] = read_can_message(msg.data, CMD_API_STAT0)
            elif api == CMD_API_STAT1:
                self.MOTOR_CURT[index] = read_can_message(msg.data, CMD_API_STAT1)
            elif api == CMD_API_STAT2:
                self.CURR_POS[index] = read_can_message(msg.data, CMD_API_STAT2)


    def CAN_recv_node(self): 	# Queue size & rate not calibrated
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
                self.MOTOR_CURT[index] = read_can_message(msg.data, CMD_API_STAT1)

                # Publish data
                motor_curt_msg = Float32MultiArray()
                motor_curt_msg.data = self.MOTOR_CURT
                self.Motor_pub.publish(motor_curt_msg)
                
            elif api == CMD_API_STAT2:
                self.CURR_POS[index] = read_can_message(msg.data, CMD_API_STAT2)

                # Publish data
                curr_angles_msg = Float32MultiArray() 
                curr_angles_msg.data = self.CURR_POS
                self.Angles_pub.publish(curr_angles_msg)


            # Control rate
            rate.sleep()


if __name__=="__main__":
    # Initialize node
    rospy.init_node('CAN_Recv', anonymous=True)

    # Setup and run node
    CAN = CAN_Recv_Node()

    # Spin to keep node alive
    rospy.spin() # Not sure this is actually needed since infinite loop is used to parse CAN messages
