"""New manual joystick script
Remaking GetManualJoystickFinal"""

# Imports
import pygame
import os
import numpy as np

# Refer to GetManualJoystickFinal, arm_master_control

# Constants
BUTTON_NAMES = ["X", "CIRCLE", "TRIANGLE", "SQUARE", "L1", 
                "R1", "L2", "R2", "SELECT", "START", "PLAY_STATION", 
                "L3", "R3", "UP", "DOWN", "LEFT", "RIGHT"]
JOYSTICK_AXES_NAMES = ["L-Right", "L-Down", "L2", "R-Right", "R-Down", "R2"]
joypos = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

def initializeJoystick():
    """Connect to joystick"""
    pygame.init()
    global joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("Initialized: ", joystick.get_name())

def getButtons():
    """Get a dictionary of button positions
    1 = pressed, 0 = not pressed/released, -1 = released
    Copied from arm_master_control"""
    pygame.event.pump()

    global buttonsPressed

    buttons = {"X": 0, "CIRCLE": 0, "TRIANGLE": 0, "SQUARE": 0 , "L1": 0, "R1": 0, "L2": 0, "R2": 0, "SELECT": 0, "START": 0, "PLAY_STATION": 0, 
        "L3": 0, "R3": 0,"UP": 0, "DOWN": 0, "LEFT": 0, "RIGHT": 0 }
    
    for i in range(0, joystick.get_numbuttons()):
        button = joystick.get_button(i)
        
        if buttonsPressed[BUTTON_NAMES[i]] == True and button == 0: # button just released
            button = -1

        if button != 1:
            buttonsPressed[BUTTON_NAMES[i]] = False
        else:
            buttonsPressed[BUTTON_NAMES[i]] = True
    
        buttons[BUTTON_NAMES[i]] = button
    
    return buttons


def getJoystickAxes():
    """Get a dictionary of joystick pushes
    Positive = same direction, negative = against
    Copied from arm_master_control"""
    pygame.event.pump()

    global joystick

    axes = {}

    #Read input from the joystick       
    for i in range(0, joystick.get_numaxes()):
        axes[JOYSTICK_AXES_NAMES[i]] = joystick.get_axis(i) 

        # set inputs to zero if near zero
        if abs(axes[JOYSTICK_AXES_NAMES[i]]) <= 0.05: 
            axes[JOYSTICK_AXES_NAMES[i]] = 0

        # zero bumpers
        if JOYSTICK_AXES_NAMES[i] == "L2" or JOYSTICK_AXES_NAMES[i] == "R2":
            axes[JOYSTICK_AXES_NAMES[i]] = (axes[JOYSTICK_AXES_NAMES[i]] + 1)/2

    return axes

    
def getManualJoystick():
    """Update the joypos based on button presses/releases and analog"""
    for game_event in pygame.event.get():
        if game_event.type == pygame.QUIT:
            # Stop running if quit input is recieved
            running = False
        elif game_event.type == pygame.KEYDOWN:
            # Else, continue
            pass
        # Get dictionaries
        buttons = getButtons()
        axes = getJoystickAxes()
        # Pressed buttons
        # Double-check the mapping, not sure
        if buttons["START"] == 1:
            # Kill switch
            joypos[29] = 1
        if buttons["TRIANGLE"] == 1:
            # Swaps (1 -> 0, 0 -> 1)
            joypos[7] = not joypos[7]
        if buttons["SQUARE"] == 1:
            joypos[5] = -1
        if buttons["L1"] == 1:
            joypos[5] = 1
        if buttons["CIRCLE"] == 1:
            joypos[6] = 1
        if buttons["X"] == 1:
            joypos[6] = -1
        # Released buttons
        if (buttons["SQUARE"] == -1 or buttons["L1"] == -1):
            joypos[5] = 0
        if (buttons["CIRCLE"] == -1 or buttons["X"] == -1):
            joypos[6] = 0
        # Analog 
        if axes["R-Down"] in (0, -1):
            joypos[3] = -(axes["R-Down"])
        if (joypos[3] < .18 and joypos[3] > -.18):
            joypos[3] = 0
        if axes["R-Right"] != 0:
            joypos[2] = axes["R-Right"]
        if (joypos[2] < .18 and joypos[2] > -.18 and axes["R-Right"] != 0):
            joypos[2] = 0
        if axes["L-Down"] != 0:
            joypos[1] = axes["L-Down"]
        if (joypos[1] < .18 and joypos[1] > -.18):
            joypos[1] = 0
        if axes["L-Right"]:
            joypos[0] = axes["L-Right"]
        if (joypos[0] < .18 and joypos[0] > -.18):
            joypos[0] = 0
        if axes["R2"] > -1:
            joypos[4] = ((axes["R2"] + 1)/ 2)
        elif (axes["L-Down"] > -1 and axes["L-Down"] != 0):
            joypos[4] = -((axes["L-Down"] + 1)/ 2)
        if (joypos[4] < .5 and joypos[4] > -.5):
            joypos[4] = 0
        
        # Return joystick positions
        return joypos
        


#initializeJoystick()
