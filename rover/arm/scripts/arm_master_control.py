#!/usr/bin/env python3

import rospy
import pygame
import numpy as np
import ik_library as ik
import arm_simulation_control as sim
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float32MultiArray
import json
import copy
from math import pi
from enum import Enum
import concurrent.futures

# Enums

class Mode(Enum):
    DEFAULT_IK = 1 # moves relative to global xyz but frame rotation is input as rpy
    GLOBAL_IK = 2 # moves and rotates relative to global xyz
    ROT_RELATIVE_IK = 3 # moves globally but rotates relative to end effector
    RELATIVE_IK = 4 # moves relative to camera (has same frame as end effector) but rotates relative to end effector (we lose)
    CAM_RELATIVE_IK = 5 # Moves and rotates relative to camera, will have to change IK target to camera location
    FORWARD_KIN = 6 # forward kinematics

# Global Variables 

BUTTON_NAMES = ["X", "CIRCLE", "TRIANGLE", "SQUARE", "L1", "R1", "L2", "R2", "SELECT", "START", "PLAY_STATION", "L3", "R3", "UP", "DOWN", "LEFT", "RIGHT"]
JOYSTICK_AXES_NAMES = ["L-Right", "L-Down", "L2", "R-Right", "R-Down", "R2"]

global gazebo_on
global curArmAngles
global scriptMode
global prevTargetTransform
global prevTargetValues # [roll, pitch, yaw, [x, y, z]]
global newTargetValues
global jointPublisher
global gazeboPublisher
global pubThreadPool

movementSpeed = 1

buttonsPressed = {"X": False, "CIRCLE": False, "TRIANGLE": False, "SQUARE": False, "L1": False, "R1": False, "L2": False, "R2": False, "SELECT": False, "START": False, "PLAY_STATION": False, 
        "L3": False, "R3": False,"UP": False, "DOWN": False, "LEFT": False, "RIGHT": False} 

# Joystick Controller

def initializeJoystick():
    '''### Connects Joystick to Script

    For when the arm is operated by a PS3, PS4, or PS5 joystick. Probably will work with X-box Controllers.
    '''
    pygame.init()
    global joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print('Initialized joystick: %s' % joystick.get_name())
    print(joystick.get_numbuttons())
    
def getJoystickButtons() -> dict: # setting up the buttons
    ''' Gets the Currently Pressed Buttons on Joystick

    If the value in the returned dictionary for a button is 
        2: button was just pressed
        1: button is pressed
        0: button is not pressed
        -1: button was just released 

    Returns
    -------
    dictionary
        dictionary with keys for buttons from BUTTON_NAMES, value is either 2, 1, 0, or -1
    
    '''
    global buttonsPressed
    global joystick

    pygame.event.pump() # allow pygame to handle internal actions, keep everything current
    
    
    buttons = {"X": 0, "CIRCLE": 0, "TRIANGLE": 0, "SQUARE": 0 , "L1": 0, "R1": 0, "L2": 0, "R2": 0, "SELECT": 0, "START": 0, "PLAY_STATION": 0, 
        "L3": 0, "R3": 0,"UP": 0, "DOWN": 0, "LEFT": 0, "RIGHT": 0 } # 1 is pressed, 0 is not pressed, -1 is just released
 
    for i in range(0, joystick.get_numbuttons()):
        button = joystick.get_button(i)
        
        if buttonsPressed[BUTTON_NAMES[i]] == True and button == 0: # button just released
            button = -1
        if buttonsPressed[BUTTON_NAMES[i]] == False and button == 1: # button just pressed
            button = 2

        if button < 1:
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

        # zero bumpers
        if JOYSTICK_AXES_NAMES[i] == "L2" or JOYSTICK_AXES_NAMES[i] == "R2":
            axes[JOYSTICK_AXES_NAMES[i]] = (axes[JOYSTICK_AXES_NAMES[i]] + 1)/2

    return axes

# Arm Control

def controlEEPosition(isButtonPressed, joystickAxis, prevTargetValues, prevTargetTransform, scriptMode):
    ''' Changes End Effector Position Matrix
    
    Takes Joystick Input and changes End Effector Transform Matrix accordingly. 
    Returns the current end effector transform if none of the relavent control buttons 
    have been pushed. Additionally, uses scriptMode to determine if end effector transform 
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
    global newTargetValues
    newTarget = copy.deepcopy(prevTargetValues)
    
    positionScale = 10
    angleScale = 0.1

    newRoll = newTarget[0]
    newPitch = newTarget[1]
    newYaw = newTarget[2]
    
    newX = newTarget[3][0]
    newY = newTarget[3][1]
    newZ = newTarget[3][2]

    if scriptMode == Mode.DEFAULT_IK:
        # control x, y, z
        if joystickAxis["L-Down"] != 0:
            newX -= joystickAxis["L-Down"]*positionScale
        if joystickAxis["L-Right"] != 0:
            newY -= joystickAxis["L-Right"]*positionScale
        if joystickAxis["L2"] != 0:
            newZ -= joystickAxis["L2"]*positionScale
        if joystickAxis["R2"] != 0:
            newZ += joystickAxis["R2"]*positionScale
        
        # control roll, pitch, yaw
        if joystickAxis["R-Right"] != 0:
            newYaw -= joystickAxis["R-Right"]*angleScale
        # if joystickAxis["R-Down"] != 0:
        #     newPitch -= joystickAxis["R-Down"]*scale2
        if isButtonPressed["DOWN"]:
            newPitch -= angleScale
        elif isButtonPressed["UP"]:
            newPitch += angleScale
        if isButtonPressed["L1"]:
            newRoll -= angleScale
        elif isButtonPressed["R1"]:
            newRoll += angleScale

        newTarget = [newRoll, newPitch, newYaw, [newX, newY, newZ]]
        # print("x: ", newX, " y: ", newY, " z: ", newZ, " roll: ", newRoll, " pitch: ", newPitch, " yaw: ", newYaw)
        newTargetValues = newTarget
        
        return ik.createEndEffectorTransform(newRoll, newPitch, newYaw, newTarget[3])
    elif scriptMode == Mode.GLOBAL_IK:
        newRoll = 0
        newPitch = 0
        newYaw = 0

        if joystickAxis["L-Down"] != 0:
            newX -= joystickAxis["L-Down"]*positionScale
        if joystickAxis["L-Right"] != 0:
            newY -= joystickAxis["L-Right"]*positionScale
        if joystickAxis["L2"] != 0:
            newZ -= joystickAxis["L2"]*positionScale
        if joystickAxis["R2"] != 0:
            newZ += joystickAxis["R2"]*positionScale
        # control roll, pitch, yaw
        if joystickAxis["R-Right"] != 0:
            newRoll += joystickAxis["R-Right"]*angleScale
        if joystickAxis["R-Down"] != 0:
            newPitch += joystickAxis["R-Down"]*angleScale
        # if isButtonPressed["UP"]:
        #     newPitch = -angleScale
        # elif isButtonPressed["DOWN"]:
        #     newPitch = angleScale
        if isButtonPressed["L1"]:
            newYaw = -angleScale
        elif isButtonPressed["R1"]:
            newYaw = angleScale
        
        prevRotation = prevTargetTransform[:3, :3]
        
        newRotation = np.dot(ik.createRotationMatrix(newRoll, newPitch, newYaw, "ypr"), prevRotation)
        newTransformation = np.block([
                                        [newRotation, np.array([[newX, newY, newZ]]).transpose()],
                                        [0, 0, 0, 1]
                                        ])
        [correctedRoll, correctedPitch, correctedYaw] = ik.calculateRotationAngles(newTransformation)

        newTargetValues = [correctedRoll, correctedPitch, correctedYaw, [newX, newY, newZ]]
        # print("x: ", newX, " y: ", newY, " z: ", newZ, " roll: ", correctedRoll, " pitch: ", correctedPitch, " yaw: ", correctedYaw)
        
        return newTransformation
    elif scriptMode == Mode.ROT_RELATIVE_IK:
        newRoll = 0
        newPitch = 0
        newYaw = 0

        if joystickAxis["L-Down"] != 0:
            newX -= joystickAxis["L-Down"]*positionScale
        if joystickAxis["L-Right"] != 0:
            newY -= joystickAxis["L-Right"]*positionScale
        if joystickAxis["L2"] != 0:
            newZ -= joystickAxis["L2"]*positionScale
        if joystickAxis["R2"] != 0:
            newZ += joystickAxis["R2"]*positionScale
        # control roll, pitch, yaw
        if joystickAxis["R-Right"] != 0:
            newRoll = -joystickAxis["R-Right"]*angleScale
        if joystickAxis["R-Down"] != 0:
            newPitch = joystickAxis["R-Down"]*angleScale
        if isButtonPressed["L1"]:
            newYaw = -angleScale
        elif isButtonPressed["R1"]:
            newYaw = angleScale
        
        prevRotation = prevTargetTransform[:3, :3]
        
        newRotation = np.matmul(prevRotation, ik.createRotationMatrix(newRoll, newPitch, newYaw, "ypr"))
        newTransformation = np.block([
                                        [newRotation, np.array([[newX, newY, newZ]]).transpose()],
                                        [0, 0, 0, 1]
                                        ])
        [correctedRoll, correctedPitch, correctedYaw] = ik.calculateRotationAngles(newTransformation)

        newTargetValues = [correctedRoll, correctedPitch, correctedYaw, [newX, newY, newZ]]

        return newTransformation
    elif scriptMode == Mode.RELATIVE_IK:
        [newRoll, newPitch, newYaw, newX, newY, newZ] = [0, 0, 0, 0, 0, 0]
        
        if joystickAxis["L-Down"] != 0:
            newX = joystickAxis["L-Down"]*positionScale
        if joystickAxis["L-Right"] != 0:
            newY = -joystickAxis["L-Right"]*positionScale
        if joystickAxis["L2"] != 0:
            newZ = -joystickAxis["L2"]*positionScale
        if joystickAxis["R2"] != 0:
            newZ = joystickAxis["R2"]*positionScale
        # control roll, pitch, yaw
        if joystickAxis["R-Right"] != 0:
            newRoll = -joystickAxis["R-Right"]*angleScale
        if joystickAxis["R-Down"] != 0:
            newPitch = joystickAxis["R-Down"]*angleScale
        if isButtonPressed["L1"]:
            newYaw = -angleScale
        elif isButtonPressed["R1"]:
            newYaw = angleScale
        
        relativePos = ik.createEndEffectorTransform(newRoll, newPitch, newYaw, [newX, newY, newZ])
        newTransformation = np.matmul(prevTargetTransform, relativePos)

        [correctedRoll, correctedPitch, correctedYaw] = ik.calculateRotationAngles(newTransformation)
        [correctedX, correctedY, correctedZ] = [newTransformation[0][3], newTransformation[1][3], newTransformation[2][3]]
        newTargetValues = [correctedRoll, correctedPitch, correctedYaw, [correctedX, correctedY, correctedZ]]

        return newTransformation
    elif scriptMode == Mode.CAM_RELATIVE_IK:
        pass
    elif scriptMode == Mode.FORWARD_KIN:
        pass

def controlGripperAngle(isButtonPressed, curArmAngles):
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
    gripperAngle = curArmAngles[6]
    # Tune amount that angle is changed with each cycle
    angleIncrement = 0.1

    # If both buttons are pressed at the same time, angle will not change
    if isButtonPressed["SQUARE"] == 1 and isButtonPressed["X"] != 1:
        gripperAngle += angleIncrement

    if isButtonPressed["X"] == 1 and isButtonPressed["SQUARE"] != 1:
        gripperAngle -= angleIncrement

    # if gripperAngle < -0.04:
    #     gripperAngle = -0.04
    # if gripperAngle > 0:
    #     gripperAngle = 0

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
    "title":positionName,
    "angle0":curArmAngles[0],
    "angle1":curArmAngles[1],
    "angle2":curArmAngles[2],
    "angle3":curArmAngles[3],
    "angle4":curArmAngles[4],
    "angle5":curArmAngles[5],
    "gripperAngle":curArmAngles[6]
    }

    with open('arm_angles.json','r+') as file:
        breakLoop = False
        savedPos = json.load(file)
    # Checks if any existing positions have same positionName
        for x in range(len(savedPos["position_names"])):
            if breakLoop:
                break
            if savedPos["position_names"][x]["title"] == positionName: # if found, deletes existing entry
                savedPos["position_names"].pop(x)
                breakLoop = True
    # Appends new position to end of file
        savedPos["position_names"].append(newAngles)
        file.seek(0)
        json.dump(savedPos,file,indent = 0)
        file.close()
    pass

def goToPosition():
    ''' Pulls up GUI with options of saved joint angles.

    '''
    retrievePos = input("Enter Position Name to Retrieve: ")

    with open('arm_angles.json','r') as file:
        found = False
        savedPos = json.load(file)
    # Checks to see if position names match request
        for x in range(len(savedPos["position_names"])):
            if savedPos["position_names"][x]["title"] == retrievePos: # if found, changes current angles to requested position
                found = True
                curArmAngles[0] = savedPos["position_names"][x]["angle0"]
                curArmAngles[1] = savedPos["position_names"][x]["angle1"]
                curArmAngles[2] = savedPos["position_names"][x]["angle2"]
                curArmAngles[3] = savedPos["position_names"][x]["angle3"]
                curArmAngles[4] = savedPos["position_names"][x]["angle4"]
                curArmAngles[5] = savedPos["position_names"][x]["angle5"]
                curArmAngles[6] = savedPos["position_names"][x]["gripperAngle"]

        if not found:
            print("Position name '"+retrievePos+"' not found") # if not found, prints message
    pass

# ROS Stuff

def publishNewAngles(newJointAngles):
    ''' Publishes the new angles to /arm_target_angles

    Publishes a list 32 bit floats.

    Parameters
    ---------- 
    newJointAngles
        an array of the new angles (7 elements)

    '''
    ikAngles = Float32MultiArray()
    ikAngles.data = newJointAngles
    armAngles.publish(ikAngles)

def updateLiveArmSimulation(data):
    ''' Updates RViz URDF visulaztion based on arm angles

    Should be used as the callback function of the subscriber. However, can
    also be used as desired angle visualizer as well.

    Paramters
    ---------
    data
        data.data containts a list 32 bit floats that correspond to joint angles
    '''
    curArmAngles = data
    curArmAngles.append(curArmAngles[6]) # make gripper angles equal
    curArmAngles.append(curArmAngles[6]) # make gripper angles equal
    if gazebo_on:
        curArmAngles[2] = curArmAngles[2] - pi/2 # adjustment for third joint (shifted 90 degrees so it won't collide with ground on startup)
        # curArmAngles[6] = -curArmAngles[6]
        print(curArmAngles)
        sim.moveInGazebo(gazeboPublisher, curArmAngles)
    else:
        sim.runNewJointState2(jointPublisher, curArmAngles)

def updateDesiredEETransformation(newTargetValues, scriptMode):
    ''' Updates tf2 transformation of desired end effector position.

    Displays a tf2 transform of the end effector position.
    
    Paramters
    ---------
    newTargetValues:
        [roll, pitch, yaw, [x, y, z]] of the position
    scriptMode
        the mode that the scriptp is in
    '''
    global jointPublisher
    global gazeboPublisher

    # the following is specific the URDF, you will have to change these values and maybe swap coordiantes if you change URDFs
    tempTarget = copy.deepcopy(newTargetValues) # target transform scaled to rviz
    tempX = tempTarget[3][0] # swap x and y coords
    tempTarget[3][0] = tempTarget[3][1]
    tempTarget[3][1] = tempX

    # scale target transform
    if tempTarget[3][0] >= 0:
        tempTarget[3][0] = -0.00101165*tempTarget[3][0] + 0.0594
    else:   
        tempTarget[3][0] = -0.00101165*tempTarget[3][0] + 0.0494
    if tempTarget[3][1] >= 0:
        tempTarget[3][1] = 0.001025*tempTarget[3][1] + 0.03
    else:
        tempTarget[3][1] = 0.001025*tempTarget[3][1] + 0.04
    tempTarget[3][2] = (0.47/450)*tempTarget[3][2]

    tempRoll = tempTarget[0] # swap roll and pitch
    tempTarget[0] = -tempTarget[1]
    tempTarget[1] = tempRoll # should be 0 for default

    referenceLink = "base_link" # link to base end effector transform off of

    if scriptMode == Mode.DEFAULT_IK:
        tempTarget[1] = 0 
    if scriptMode == Mode.GLOBAL_IK:
        tempTarget[:3] = [0, 0, 0]
    if scriptMode == Mode.ROT_RELATIVE_IK or scriptMode == Mode.RELATIVE_IK:
        tempTarget = [0, 0 , pi, [0, 0, 150*0.47/450]]
        referenceLink = "Link_6"
    
    sim.displayEndEffectorTransform(tempTarget, referenceLink) # display target transform

# Program Control

def main():
    ''' Function that runs the program
    
    Combines all code to takes Joystick values, chooses script mode, changes arm movement speed,
    changes EE transform, solves IK, publishes joints, and updates RViz.
    '''

    global curArmAngles
    global scriptMode
    global prevTargetTransform
    global prevTargetValues
    global newTargetValues
    global pubThreadPool
    
    isButtonPressed = getJoystickButtons()
    joystickAxes = getJoystickAxes()

    if isButtonPressed["TRIANGLE"] == 2: # changes mode when button is released
        if (scriptMode.value + 1) > len(Mode):
            scriptMode = list(Mode)[0]
        else:
            scriptMode = list(Mode)[scriptMode.value]
        print(scriptMode.name)

    if scriptMode.value <= 4: # everything below CAM_RELATIVE_MOTION
        dhTable = ik.createDHTable(curArmAngles)

        targetEEPos = controlEEPosition(isButtonPressed, joystickAxes, prevTargetValues, 
                                        prevTargetTransform, scriptMode)
        
        try:
            targetAngles = ik.inverseKinematics(dhTable, targetEEPos) 
            targetAngles.append(controlGripperAngle(isButtonPressed, curArmAngles))

            # publish on different threads to speed up processing time
            pubThreadPool.submit(publishNewAngles, targetAngles)
            pubThreadPool.submit(updateDesiredEETransformation, newTargetValues, scriptMode)
            pubThreadPool.submit(updateLiveArmSimulation, targetAngles)

            curArmAngles = targetAngles
            prevTargetTransform = targetEEPos
            prevTargetValues = newTargetValues
        except ik.CannotReachTransform:
            print("Cannot reach outside of arm workspace")
            pass
    elif scriptMode == Mode.FORWARD_KIN:
        pass

# Main Area

if __name__ == "__main__":
    curArmAngles = [0, 0, 0, 0, 0, 0, 0]
    prevTargetValues = [0, 0, 0, [250, 0, 450]] # start position
    newTargetValues = prevTargetValues
    prevTargetTransform = ik.createEndEffectorTransform(prevTargetValues[0], prevTargetValues[1],
                                                        prevTargetValues[2], prevTargetValues[3])
    
    initializeJoystick()

    pubThreadPool = concurrent.futures.ThreadPoolExecutor(max_workers=2) # create thread pool with three threads

    # try:
    scriptMode = Mode.GLOBAL_IK
    print(scriptMode.name)

    rospy.init_node("arm_master_control")
    gazebo_on = rospy.get_param("/gazebo_on")
    rate = rospy.Rate(10) # run at 10Hz

    armAngles = rospy.Publisher("ik_angles", Float32MultiArray, queue_size=10)
    rospy.Subscriber("arm_angles", Float32MultiArray, updateLiveArmSimulation)

    if gazebo_on:
        gazeboPublisher = sim.startGazeboJointControllers(9)
    else:
        jointPublisher = sim.startJointPublisher()

    # sets start target position equal to curArmAngles after they have been updated
    # while curArmAngles == [0, 0, 0, 0, 0, 0]:
    #     tempDHTable = ik.createDHTable(curArmAngles)
    #     prevTargetPos = ik.calculayteTransformToLink(tempDHTable, 5)

    while not rospy.is_shutdown():
        main()
        rate.sleep()

    # except Exception as ex:
    #     print("The following error has occured: ")
    #     print(ex)
