#!/usr/bin/env python3

import rospy
import pygame
import numpy
import ik_library as ik
import arm_simulation_control as sim
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float32MultiArray
import json

# Global Variables 

BUTTON_NAMES = ["X", "CIRCLE", "TRIANGLE", "SQUARE", "L1", "R1", "L2", "R2", "SELECT", "START", "PLAY_STATION", "L3", "R3", "UP", "DOWN", "LEFT", "RIGHT"]
JOYSTICK_AXES_NAMES = ["L-Right", "L-Down", "L2", "R-Right", "R-Down", "R2"]

global curArmAngles
global ikMode # "global" or "relativeCamera"
global prevTargetPos

movementSpeed = 1

buttonsPressed = {"X": False, "CIRCLE": False, "TRIANGLE": False, "SQUARE": False, "L1": False, "R1": False, "L2": False, "R2": False, "SELECT": False, "START": False, "PLAY_STATION": False, 
        "L3": False, "R3": False,"UP": False, "DOWN": False, "LEFT": False, "RIGHT": False} 

# Joystick Controller

def initializeJoystick():
    '''### Connects Joystick to Script

    For when the arm is operated by a PS3 or PS4 joystick. Could work with X-box Controllers.
    '''
    pygame.init()
    global joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print('Initialized joystick: %s' % joystick.get_name())
    # print(joystick.get_numbuttons())
    
def getJoystickButtons() -> dict: # setting up the buttons
    ''' Gets the Currently Pressed Buttons on Joystick

    If the value in the returned dictionary for a button is 
        1: button is pressed
        0: button is not pressed
        -1: button was just released 

    Returns
    -------
    dictionary
        dictionary with keys for buttons from BUTTON_NAMES, value is either 1, 0, or -1
    
    '''
    pygame.event.pump() # allow pygame to handle internal actions, keep everything current
    
    global buttonsPressed
    
    buttons = {"X": 0, "CIRCLE": 0, "TRIANGLE": 0, "SQUARE": 0 , "L1": 0, "R1": 0, "L2": 0, "R2": 0, "SELECT": 0, "START": 0, "PLAY_STATION": 0, 
        "L3": 0, "R3": 0,"UP": 0, "DOWN": 0, "LEFT": 0, "RIGHT": 0 } # 1 is pressed, 0 is not pressed, -1 is just released
 
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

def getJoystickAxes(): # setting up the axes for the control stick
    '''Get the state of the controllers joystick axes and bumpers.

    Get the value of joystick axes that are labelled in JOYSTICK_AXES_NAMES. Anything pushed
    in the direction labelled will be returned as positive float; against will be negative. L2 
    and R2 swill return depth pushed in.
    
    Returns
    -------
    #### dictionary: 
    A dictionary with keys that are from JOYSTICK_AXES_NAME, value is amount pushed.
    '''
    
    global joystick
    
    axes = {}
    
    pygame.event.pump()
    #Read input from the joystick       
    for i in range(0, joystick.get_numaxes()):
        axes[JOYSTICK_AXES_NAMES[i]] = joystick.get_axis(i)

        # set inputs to zero if near zero
        if abs(axes[JOYSTICK_AXES_NAMES[i]]) <= 0.05: 
                axes[JOYSTICK_AXES_NAMES[i]] = 0
    
    return axes

# Arm Control

def controlEEPosition(currentEETransform, isButtonPressed, joystickAxis):
    ''' Changes End Effector Position Matrix
    
    Takes Joystick Input and changes End Effector Transform Matrix accordingly. 
    Returns the current end effector transform if none of the relavent control buttons 
    have been pushed. Additionally, uses ikMode to determine if end effector transform 
    should be changed relative to global coordinate system or from camera coordinate system.
    Uses ik.createEndEffectorTransform().

    Paramters
    ---------
    currentEETransform
        numpy matrix of current end effector transform

    isButtonPressed
        dictionary that contains button status with key names from BUTTON_NAMES constant
    
    joystickAxis
        dictionary that contains floats that represent magnitude of X and Y directions that each joytstick is pointed in. 
        Also contains depth that triggers are pushed in. Values can be retrived via joystickAxis["Axis Name"].

    Returns
    -------
    numpy matrix
        numpy matrix of new end effector position 
    '''
    global ikMode

    pass

def controlGripperAngle(isButtonPressed):
    ''' Adjust the angle at which the gripper is open

    Moves all fingers by an equal amount. Focuses on L1 and L2.

    Paramters
    ---------
    isButtonPressed
        a dictionary containing buttons pressed. Values can be retrived via isButtonPressed["Button Name"].
    
    Returns
    -------
    float
        a float containing the angle at which the gripper is open.
    
    
    '''
    global curArmAngles
    gripperAngle = curArmAngles[6]
    # Tune amount that angle is changed with each cycle
    angleIncrement = 1

    # If both buttons are pressed at the same time, angle will not change
    if isButtonPressed["L1"] == 1 and isButtonPressed["L2"] != 1:
        gripperAngle += angleIncrement

    if isButtonPressed["L2"] == 1 and isButtonPressed["L1"] != 1:
        gripperAngle -= angleIncrement

    return gripperAngle

def savePosition():
    ''' Saves the current angles the arm is in

    Uses global variable curArmAngles and saves angles to file. Asks for name
    of position before saving (preferable a GUI).
    '''
    global curArmAngles

    # (Placeholders for angles 1-5)

    positionName = input("Enter Position Name: ")
    newAngles = {
        "title:":positionName,
        "angle0":curArmAngles[0],
        "angle1":curArmAngles[1],
        "angle2":curArmAngles[2],
        "angle3":curArmAngles[3],
        "angle4":curArmAngles[4],
        "angle5":curArmAngles[5],
        "gripperAngle":curArmAngles[6]
    }

    with open('arm_angles.json','r+') as file:
        current_file = json.load(file)
        current_file["arm_angles"].append(newAngles)
        file.seek(0)
        json.dump(current_file,file,indent = 0)

def goToPosition():
    ''' Pulls up GUI with options of saved joint angles.

    '''
    pass

# ROS Stuff

def publishNewAngles(newJointAngles):
    ''' Publishes the new angles to /arm_target_angles

    Publishes a list 32 bit floats.

    Parameters
    ---------- 
    newJointAngles
        an array of the new angles (6 elements)

    '''
    pass

def updateLiveArmSimulation(data):
    ''' Updates RViz URDF simulation based on real arm angles

    Should be used as the callback function of the subscriber. Runs code
    from arm_simulation_control.py and updates global variable armAngles.

    Paramters
    ---------
    data
        data.data containts a list 32 bit floats that correspond to joint angles
    '''
    global armAngles

    pass

def updateDesiredArmSimulation(endEffectorTransform):
    ''' Updates RViz simulation with desired arm position.

    Displays a tf2 transform of the end effector position.
    
    Paramters
    ---------
    endEffectorTransform:
        numpy matrix that corresponds to end effector transform 
    '''
    pass

# Program Control

def main():
    ''' Function that runs the program
    
    Takes Joystick values, chooses IK type (global or relative to camera), changes arm movement speed, changes EE transform, 
    solves IK, publishes joints, and updates RViz simualtion.
    '''
    global curArmAngles
    global ikMode
    global prevTargetPos
    pass


# Main Area

if __name__ == "__main__":
    curArmAngles = [0, 0, 0, 0, 0, 0]
    ikMode = "global"

    # initializeJoystick()

    
    try:
        rate = rospy.Rate(10) # run at 10Hz
        rospy.Subscriber("arm_angles", Float32MultiArray, updateLiveArmSimulation)

        # sets start target position equal to curArmAngles after they have been updated
        while curArmAngles != [0, 0, 0, 0, 0, 0]:
            tempDHTable = ik.createDHTable(curArmAngles)
            prevTargetPos = ik.calculateTransformToLink()

        while not rospy.is_shutdown():
            main()
            rate.sleep()

    except Exception as ex:
        print("The following error has occured: ")
        print(ex)
