"""New manual joystick script
Remaking GetManualJoystickFinal"""

# Imports
import pygame
import os
import numpy as np
import time
#import rospy

# Refer to GetManualJoystickFinal, arm_master_control

######################## CLASSES ##########################

class Manual_Node():
    """
    (None)
    
    A class that initializes the manual node and connects it to different topics for publishing and subscribing
    """
    def __init__(self):

        # List of buttons and axes for manual control
        self.BUTTON_NAMES        = ["X", "CIRCLE", "TRIANGLE", "SQUARE", "L1", 
                                    "R1", "L2", "R2", "SELECT", "START", "PLAY_STATION", 
                                    "L3", "R3", "UP", "DOWN", "LEFT", "RIGHT"]
        self.JOYSTICK_AXES_NAMES = ["L-Right", "L-Down", "L2", "R-Right", "R-Down", "R2"]

        # Buffer to hold input from Ros Topic
        self.joypos              = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # List containing goal_pos that needs to be published on the ROs topic
        self.controller_pos      = [0, 0, 0, 0, 0, 0, 0, 0]

        # Manual Speed limits, values are given by trial and error
        self.speed_limit         = [1000/120, 1000/160, 4000/120, 2000/20, 1500/20, 1500/20, 10000/40] # Gear Ratio are after the /

        # Variable to hold time
        self.t                   = 0

        # Variable to hold the status 
        self.status = "Idle"

        # Connecting to Ros topics by subsribing or publishing
        #self.error               = rospy.Subscriber("Error_msg", daa type, )
        #self.input               = rospy.Subscriber("Input", daa type, )
        #self.state               = rospy.Subscriber("State", daa type, )
        #self.goal                = rospy.Publisher("Goal_pos", )

        # preparing controller_pos values based on the state
        if self.state == "Manual":
            #self.getManualJoystick()
            self.MapJoystick(self.joypos, self.controller_pos, self.speed_limit, time.time() - self.t)
        if self.state == "Setup":
            self.setup()
        
    def SetJointSpeed(self, joypos : list, controller_pos : list, speed_limit : list, dt : float) -> list:
        """
        (list(float), list(float), list(float), float) -> list(float)

        Takes in controller inputs and updates the controller angles (in degrees)

        @parameters

        joypos (list(float)): Buffer holding the input from controller
        
        controller_pos (list(float)): List containing the current angles (in degrees) of the motors
        
        speed_limit (list(float)): List containing tested out manually controlled speeds for each motor
        
        dt (float): Time since last call 
        """
        # Calculate joint speed from speed limit, joypos, time taken, and controller pos
        return ((speed_limit*(joypos)*dt) + controller_pos)
    
        
    def MapJoystick(self, joypos : list, controller_pos : list, speed_limit : list, dt : list):   
        """
        (list(float), list(float), list(float), float) -> list(float)
        Takes in joypos, motor positions, and speed limits and updates motor positions

        @parameters
        joypos (list(float)): Buffer holding the input from controller
        
        controller_pos (list(float)): List containing the current angles (in degrees) of the motors
        
        speed_limit (list(float)): List containing tested out manually controlled speeds for each motor
        
        dt (float): Time since last call
        """

        # Set controller pos based on joint speed calculations/joypos
        controller_pos[0] = self.SetJointSpeed(joypos[0], controller_pos[0], speed_limit[0], dt)
        controller_pos[1] = self.SetJointSpeed(-joypos[1], controller_pos[1], speed_limit[1], dt)
        controller_pos[2] = self.SetJointSpeed(joypos[3], controller_pos[2], speed_limit[2], dt)
        controller_pos[3] = self.SetJointSpeed(joypos[2], controller_pos[3], speed_limit[3], dt)
        controller_pos[4] = self.SetJointSpeed(joypos[4], controller_pos[4], speed_limit[4], dt)
        controller_pos[5] = self.SetJointSpeed(joypos[5], controller_pos[5], speed_limit[5], dt)
        controller_pos[6] = self.SetJointSpeed(joypos[6], controller_pos[6], speed_limit[6], dt)
        controller_pos[7] = joypos[7]

        # Measure time it takes to update controller pos
        self.t            = time.time()
        return controller_pos
    
    def setup(self):
        # For each motor/controller pos...
        for i in range(len(self.controller_pos)):
            # Send in a large value to hit limit switch
            self.controller_pos[i] = 10000000
            # When error occurs (recieve from safety node), set it back to zero
            # Currently assuming error messages will be str
            while "LIMIT_SWITCH" not in self.error:
                pass
            self.controller_pos[i] = 0
        pass
        #a variable correction value

############################## MAIN ############################

def main():
    #rospy.init_node("Arm_Manual", anonymous= True)
    
    Node_Manual = Manual_Node()

    #rospy.spin()

    pass


if __name__ == "__main__":

    main()

#initializeJoystick()
    
    # Joypos updates will be handled by controller

    # def getManualJoystick(self):
    #     """
    #     Takes in buttons and analog inputs and updates joypos

    #     @parameters

    #     None
    #     """
    #     for game_event in pygame.event.get():
    #         if game_event.type == pygame.QUIT:
    #             # Stop running if quit input is recieved
    #             running = False
    #         elif game_event.type == pygame.KEYDOWN:
    #             # Else, continue
    #             pass
    #         # Get dictionaries
    #         buttons = getButtons()
    #         axes = getJoystickAxes()
    #         # Pressed buttons
    #         # Double-check the mapping, not sure
    #         if buttons["START"] == 1:
    #             # Kill switch
    #             self.joypos[29] = 1
    #         if buttons["TRIANGLE"] == 1:
    #             # Swaps (1 -> 0, 0 -> 1)
    #             self.joypos[7] = not self.joypos[7]
    #         if buttons["SQUARE"] == 1:
    #             self.joypos[5] = -1
    #         if buttons["L1"] == 1:
    #             self.joypos[5] = 1
    #         if buttons["CIRCLE"] == 1:
    #             self.joypos[6] = 1
    #         if buttons["X"] == 1:
    #             self.joypos[6] = -1
    #         # Released buttons
    #         if (buttons["SQUARE"] == -1 or buttons["L1"] == -1):
    #             self.joypos[5] = 0
    #         if (buttons["CIRCLE"] == -1 or buttons["X"] == -1):
    #             self.joypos[6] = 0
    #         # Analog 
    #         if axes["R-Down"] in (0, -1):
    #             self.joypos[3] = -(axes["R-Down"])
    #         if (self.joypos[3] < .18 and self.joypos[3] > -.18):
    #             self.joypos[3] = 0
    #         if axes["R-Right"] != 0:
    #             self.joypos[2] = axes["R-Right"]
    #         if (self.joypos[2] < .18 and self.joypos[2] > -.18 and axes["R-Right"] != 0):
    #             self.joypos[2] = 0
    #         if axes["L-Down"] != 0:
    #             self.joypos[1] = axes["L-Down"]
    #         if (self.joypos[1] < .18 and self.joypos[1] > -.18):
    #             self.joypos[1] = 0
    #         if axes["L-Right"]:
    #             self.joypos[0] = axes["L-Right"]
    #         if (self.joypos[0] < .18 and self.joypos[0] > -.18):
    #             self.joypos[0] = 0
    #         if axes["R2"] > -1:
    #             self.joypos[4] = ((axes["R2"] + 1)/ 2)
    #         elif (axes["L-Down"] > -1 and axes["L-Down"] != 0):
    #             self.joypos[4] = -((axes["L-Down"] + 1)/ 2)
    #         if (self.joypos[4] < .5 and self.joypos[4] > -.5):
    #             self.joypos[4] = 0
            
    #         # Return joystick positions
    #         return self.joypos
    

# Constants
# BUTTON_NAMES = ["X", "CIRCLE", "TRIANGLE", "SQUARE", "L1", 
#                 "R1", "L2", "R2", "SELECT", "START", "PLAY_STATION", 
#                 "L3", "R3", "UP", "DOWN", "LEFT", "RIGHT"]
# JOYSTICK_AXES_NAMES = ["L-Right", "L-Down", "L2", "R-Right", "R-Down", "R2"]
# joypos = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# controller_pos = [0,0,0,0,0,0,0,0]

# def initializeJoystick():
#     """Connect to joystick"""
#     pygame.init()
#     global joystick
#     joystick = pygame.joystick.Joystick(0)
#     joystick.init()
#     print("Initialized: ", joystick.get_name())

# def getButtons():
#     """Get a dictionary of button positions
#     1 = pressed, 0 = not pressed/released, -1 = released
#     Copied from arm_master_control"""
#     pygame.event.pump()

#     global buttonsPressed

#     buttons = {"X": 0, "CIRCLE": 0, "TRIANGLE": 0, "SQUARE": 0 , "L1": 0, "R1": 0, "L2": 0, "R2": 0, "SELECT": 0, "START": 0, "PLAY_STATION": 0, 
#         "L3": 0, "R3": 0,"UP": 0, "DOWN": 0, "LEFT": 0, "RIGHT": 0 }
    
#     for i in range(0, joystick.get_numbuttons()):
#         button = joystick.get_button(i)
        
#         if buttonsPressed[BUTTON_NAMES[i]] == True and button == 0: # button just released
#             button = -1

#         if button != 1:
#             buttonsPressed[BUTTON_NAMES[i]] = False
#         else:
#             buttonsPressed[BUTTON_NAMES[i]] = True
    
#         buttons[BUTTON_NAMES[i]] = button
    
#     return buttons


# def getJoystickAxes():
#     """Get a dictionary of joystick pushes
#     Positive = same direction, negative = against
#     Copied from arm_master_control"""
#     pygame.event.pump()

#     global joystick

#     axes = {}

#     #Read input from the joystick       
#     for i in range(0, joystick.get_numaxes()):
#         axes[JOYSTICK_AXES_NAMES[i]] = joystick.get_axis(i) 

#         # set inputs to zero if near zero
#         if abs(axes[JOYSTICK_AXES_NAMES[i]]) <= 0.05: 
#             axes[JOYSTICK_AXES_NAMES[i]] = 0

#         # zero bumpers
#         if JOYSTICK_AXES_NAMES[i] == "L2" or JOYSTICK_AXES_NAMES[i] == "R2":
#             axes[JOYSTICK_AXES_NAMES[i]] = (axes[JOYSTICK_AXES_NAMES[i]] + 1)/2

#     return axes

    
# def getManualJoystick():
#     """Update the joypos based on button presses/releases and analog"""
#     for game_event in pygame.event.get():
#         if game_event.type == pygame.QUIT:
#             # Stop running if quit input is recieved
#             running = False
#         elif game_event.type == pygame.KEYDOWN:
#             # Else, continue
#             pass
#         # Get dictionaries
#         buttons = getButtons()
#         axes = getJoystickAxes()
#         # Pressed buttons
#         # Double-check the mapping, not sure
#         if buttons["START"] == 1:
#             # Kill switch
#             joypos[29] = 1
#         if buttons["TRIANGLE"] == 1:
#             # Swaps (1 -> 0, 0 -> 1)
#             joypos[7] = not joypos[7]
#         if buttons["SQUARE"] == 1:
#             joypos[5] = -1
#         if buttons["L1"] == 1:
#             joypos[5] = 1
#         if buttons["CIRCLE"] == 1:
#             joypos[6] = 1
#         if buttons["X"] == 1:
#             joypos[6] = -1
#         # Released buttons
#         if (buttons["SQUARE"] == -1 or buttons["L1"] == -1):
#             joypos[5] = 0
#         if (buttons["CIRCLE"] == -1 or buttons["X"] == -1):
#             joypos[6] = 0
#         # Analog 
#         if axes["R-Down"] in (0, -1):
#             joypos[3] = -(axes["R-Down"])
#         if (joypos[3] < .18 and joypos[3] > -.18):
#             joypos[3] = 0
#         if axes["R-Right"] != 0:
#             joypos[2] = axes["R-Right"]
#         if (joypos[2] < .18 and joypos[2] > -.18 and axes["R-Right"] != 0):
#             joypos[2] = 0
#         if axes["L-Down"] != 0:
#             joypos[1] = axes["L-Down"]
#         if (joypos[1] < .18 and joypos[1] > -.18):
#             joypos[1] = 0
#         if axes["L-Right"]:
#             joypos[0] = axes["L-Right"]
#         if (joypos[0] < .18 and joypos[0] > -.18):
#             joypos[0] = 0
#         if axes["R2"] > -1:
#             joypos[4] = ((axes["R2"] + 1)/ 2)
#         elif (axes["L-Down"] > -1 and axes["L-Down"] != 0):
#             joypos[4] = -((axes["L-Down"] + 1)/ 2)
#         if (joypos[4] < .5 and joypos[4] > -.5):
#             joypos[4] = 0
        
#         # Return joystick positions
#         return joypos
        


# speed_limit = [1000/120, 1000/160, 4000/120, 2000/20, 1500/20, 1500/20, 10000/40]                    # Gear Ratio are after the /
# t = 0



# def SetJointSpeed(joypos, controller_pos, speed_limit, dt):

#     return (speed_limit*(joypos)*dt)+ controller_pos



# def MapJoystick(joypos, controller_pos, speed_limit, dt):   #takes in the joystick input(I), the current position of the motors (O), the speed limits (S)

#     controller_pos[0] = SetJointSpeed(joypos[0], controller_pos[0], speed_limit[0], dt)
#     controller_pos[1] = SetJointSpeed(-joypos[1], controller_pos[1], speed_limit[1], dt)
#     controller_pos[2] = SetJointSpeed(joypos[3], controller_pos[2], speed_limit[2], dt)
#     controller_pos[3] = SetJointSpeed(joypos[2], controller_pos[3], speed_limit[3], dt)
#     controller_pos[4] = SetJointSpeed(joypos[4], controller_pos[4], speed_limit[4], dt)
#     controller_pos[5] = SetJointSpeed(joypos[5], controller_pos[5], speed_limit[5], dt)
#     controller_pos[6] = SetJointSpeed(joypos[6], controller_pos[6], speed_limit[6], dt)
#     controller_pos[7] = joypos[7]
    


#     return controller_pos
