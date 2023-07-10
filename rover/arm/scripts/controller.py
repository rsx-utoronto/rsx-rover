#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from rover.msg import ArmInputs
import arm_servo
#from rover.srv import Corrections

'''
publishes: arm_state, arm_inputs (joystick value)
subscribes: Joy
rosservice: correction
'''

class Controller():
    """
    (None)

    This class represents an instance of controller node and connects the node to 
    its publishing and subscribing topics
    """

    def __init__(self):

        ## Attributes to hold data for publishing to topics
        # Attribute to publish state
        self.state               = "Idle"

        # Attribute to publish arm inputs (with initialized values)
        self.values              = ArmInputs()
        self.values.l_horizontal = 0
        self.values.l_vertical   = 0
        self.values.r_horizontal = 0
        self.values.r_vertical   = 0
        self.values.l1r1         = 0
        self.values.l2r2         = 0
        self.values.xo           = 0
        self.values.ps           = 0

        ## Attribute to store servo state
        # state -> angle:
        # 0 -> 63 degrees
        # 1 -> 84 degrees
        self.servo               = 0

        ## Variables for ROS publishers and subscrives
        self.joy_sub             = rospy.Subscriber("joy", Joy, getROSJoy)
        self.state_pub           = rospy.Publisher("arm_state", String, queue_size=10)
        self.input_pub           = rospy.Publisher("arm_inputs", ArmInputs, queue_size=10)

    def getROSJoy(self, joy_input : Joy) -> None:    
        ''' 
        (Joy) -> (None)

        Gets joystick values from "Joy" Topic

        Assuming using this package:
            http://wiki.ros.org/joy
        
        
        @paramters
    
        joy_input (Joy): Stores the received joystick inputs from "Joy" topic 
        '''
        ## Getting the joystick inputs
        # Axes Mapping (note: max absolute value = 1)
        # Idx : Button on PS4 (direction)
        # 0 : Left Analog Stick Horizontal (left positive, right negative)
        # 1 : Left Analog Stick vertical (Up positive, down negative)
        # 2 : L2 (1 when unpressed, -1 when full pressed)
        # 3 : Right Analog Stick Horizontal (left positive, right negative)
        # 4 : Right Analog Stick vertical (Up positive, down negative)
        # 5 : R2 (1 when unpressed, -1 when full pressed)
        # 6 : D-Pad Horizontal (left Arrow = -1, Right Arrow = 1, 0 when neither are pressed)
        # 7 : D-Pad Vertical (Up Arrow = 1, Down Arrow = -1, 0 when neither are pressed)
        rawAxes                     = joy_input.axes

        # Button Mapping (0 when unpressed, 1 when pressed)
        # Idc : Button
        # 0 : X
        # 1 : O (circle)
        # 2 : triangle
        # 3 : square
        # 4 : L1
        # 5 : R1
        # 6 : L2 (even when lightly pressed, it becomes 1)
        # 7 : R2 (even when lightly pressed, it becomes 1)
        # 8 : share
        # 9 : options
        # 10: PS
        # 11: L3
        # 12: R3
        rawButtons                  = joy_input.buttons

        # Setting the ArmInputs variable to be published later
        self.values.l_horizontal    = rawAxes[0]
        self.values.l_vertical      = rawAxes[1]
        self.values.r_horizontal    = rawAxes[3]
        self.values.r_vertical      = rawAxes[4]
        self.values.l1r1            = rawButtons[4] - rawButtons[5]
        self.values.l2r2            = -0.5*(rawAxes[2] - rawAxes[5])
        self.values.xo              = rawButtons[0] - rawButtons[1]
        self.values.ps              = rawButtons[10]

        # Check if analog sticks are not moving and triggers are not pressed 
        # and any other buttons are not pressed. If any of them is false, then do not
        # change the state
        if ((not (rawAxes[0] or rawAxes[1] or rawAxes[3] or rawAxes[4])) 
            and (rawAxes[2] == 1 and rawAxes[5] == 1) and (1 not in rawButtons)):
            
            if rawAxes[7] == -1:
                self.state = "Idle"
            
            elif rawAxes[6] == 1:
                self.state = "Manual"

            elif rawAxes[7] == 1:
                self.state = "Setup"

            elif rawAxes[6] == -1:
                self.state = "IK"

        # Printing state on the console and publishing it
        print("State:", self.state)
        self.state_pub.publish(self.state)

        # If square is pressed, flip the servo configuration
        if rawButtons[3] == 1:
            self.servo = not self.servo
            
            if self.servo:
                arm_servo.write_servo_high_angle()
                print("Servo going to 84 degrees configuration")
            
            else:
                arm_servo.write_servo_low_angle()
                print("Servo going to 63 degrees configuration")

        # Print/Publish the inputs if state is neither Idle or Setup
        if self.state != "Idle" and self.state != "Setup":
            print(self.values)
            self.input_pub.publish(self.values)



    # def updateCorrections():
    #     ''' Update the Correction values the IK node is using

    #     Paramters
    #     ---------

    #     Returns
    #     -------
    #     '''

    #     try:
    #         correctionService = rospy.ServiceProxy('update_ik_corrections', Corrections)
    #         serviceResponse = correctionService() # input data into function to send to service, response contains boolean if succesfully updated or not
    #     except rospy.ServiceException as ex:
    #         print("Service call failed: %s"%ex)
 
if __name__ == "__main__":
 
    try:
        rospy.init_node("controller")
        
        Controller_Node = Controller()

        rospy.spin()

        
    except rospy.ROSInterruptException:
        pass