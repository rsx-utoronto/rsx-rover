#!/usr/bin/env python3

from CAN_utilities import *
import rospy
from std_msgs.msg import Float32MultiArray, UInt8MultiArray

class CAN_Recv():
    """
    (None)

    This class represents an instance of the CAN_Recv that connects to CAN bus and reads messages
    from it. The read messages are published onto the topics.
    """

    def __init__(self):
        #self.pub_rate           = 100000000

        # Attributes to hold data from publishers or to publish
        self.CURR_POS 			= [0, 0, 0, 0, 0, 0, 0]
        self.CURR_ANGLE         = [0, 0, 0, 0, 0, 0, 0]
        self.MOTOR_CURR 		= [0, 0, 0, 0, 0, 0, 0]
        self.LIMIT_SWITCH 		= [0, 0, 0, 0, 0, 0, 0]

        # Variables for ROS publishers and subscribers
        self.Angles_pub 		= rospy.Publisher('arm_curr_pos', Float32MultiArray, queue_size=0)
        self.Motor_pub 			= rospy.Publisher('arm_motor_curr', Float32MultiArray, queue_size=0)
        self.LMS_pub 			= rospy.Publisher('arm_limit_switch', UInt8MultiArray, queue_size=0)

        # ROS
        try:
            self.read_msgs()
        except rospy.ROSInterruptException:
            print("Exception Occured")
            pass


    def read_message_from_spark(self, msg : can.Message):
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
        
        # If every element in motor_list is True, it means this was for initialization 
        # and we should break out of the loop
        # if not False in motor_read:
        # 	break
        
        # Getting the list index value based on motor device id
        index   = dev_id - 11

        # If this is for initialization, set the correseponding element in motor_list to be True
        # if init:
        # 	motor_read[index] = True

        # API for reading limit switch
        if api == CMD_API_STAT0:

            # Update the LIMIT_SWITCH data
            self.LIMIT_SWITCH[index] = read_can_message(msg.data, CMD_API_STAT0)

        # API for reading motor current
        elif api == CMD_API_STAT1:

            # Update the MOTOR_CURR data
            self.MOTOR_CURR[index]   = read_can_message(msg.data, CMD_API_STAT1)

        # API for reading current position of motor
        elif api == CMD_API_STAT2:

            # Update the CURR_POS data
            self.CURR_POS[index]     = read_can_message(msg.data, CMD_API_STAT2, index)

            # Check if we updated wrist motors and apply the conversions
            if index == 4 or index == 5:
                wrist1_angle       = self.CURR_POS[4]
                wrist2_angle       = self.CURR_POS[5]
                self.CURR_ANGLE[4] = (wrist1_angle + wrist2_angle) / (2 * WRIST_RATIO)
                self.CURR_ANGLE[5] = (wrist1_angle - wrist2_angle) / 2
            
            else:
                self.CURR_ANGLE[index] = self.CURR_POS[index]



    def read_msgs(self):
        # Set publishing rate to 10hz
        #rate = rospy.Rate(self.pub_rate)

        # Start the infinite loop
        while not rospy.is_shutdown():

            # Read and process messages
            msg                     = BUS.recv(timeout= 1)
            if msg:
                self.read_message_from_spark(msg= msg)

                # Publish most recent limit switch data
                limit_switch_msg        = UInt8MultiArray()
                limit_switch_msg.data   = self.LIMIT_SWITCH
                self.LMS_pub.publish(limit_switch_msg)

            

                # Publish most recent motor current position data
                MOTOR_CURR_msg          = Float32MultiArray()
                MOTOR_CURR_msg.data     = self.MOTOR_CURR
                self.Motor_pub.publish(MOTOR_CURR_msg)
                
            

                # Publish most recent current position data
                curr_angles_msg         = Float32MultiArray() 
                curr_angles_msg.data    = self.CURR_ANGLE
                self.Angles_pub.publish(curr_angles_msg)


            # Control rate
            #rate.sleep()
        
        # Stop the heartbeat when ROS is killed
        task.stop()


if __name__=="__main__":

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

    # Initialize node
    rospy.init_node('CAN_Recv')

    # Setup and run node
    CAN_Recv_node = CAN_Recv()

    # Spin to keep node alive
    rospy.spin()
