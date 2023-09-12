#!/usr/bin/env python3

import rospy
import numpy as np
import ik_library as ik
import arm_simulation_control as sim
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float32MultiArray, String
from rover.msg import ArmInputs
from rover.srv import Corrections, CorrectionsResponse, GoToArmPos, GoToArmPosResponse, SaveArmPos, SaveArmPosResponse
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

NODE_RATE = 1000    
BUTTON_NAMES = ["X", "CIRCLE", "TRIANGLE", "SQUARE", "L1", "R1", "L2", "R2", "SHARE", "OPTIONS", "PLAY_STATION", "L3", "R3", "UP", "DOWN", "LEFT", "RIGHT"]
JOYSTICK_AXES_NAMES = ["L-Right", "L-Down", "L2", "R-Right", "R-Down", "R2"]
LIMIT_SWITCH_ANGLES = [0, 0, 0, 0, 0, 0, 0]
VECTOR_SMOOTHING = 0.5

global gazebo_on
global curArmAngles
global liveArmAngles
global scriptMode
global goToPosValues # [isSrvCalled, [posAngles]]
global prevTargetTransform
global prevTargetValues # [roll, pitch, yaw, [x, y, z]]
global newTargetValues
global jointPublisher
global gazeboPublisher
global angleCorrections
global isMovementNormalized
global savedCanAngles
global ikEntered

ikEntered = False
movementSpeed = 10/NODE_RATE
isMovementNormalized = False

buttonsPressed = {"X": False, "CIRCLE": False, "TRIANGLE": False, "SQUARE": False, "L1": False, "R1": False, "L2": False, "R2": False, "SHARE": False, "OPTIONS": False, "PLAY_STATION": False, 
        "L3": False, "R3": False,"UP": False, "DOWN": False, "LEFT": False, "RIGHT": False} 

class ScriptState():
    def __init__(self) -> None:
        pass

    def main() -> None:
        pass

    def onJoystickUpdate() -> None:
        pass

    def controlEEPosition() -> None:
        pass

    def updateDesiredEETrasformtion():
        pass

class ForwardKin(ScriptState):
    def main() -> None:
        global prevTargetValues
        global curArmAngles

        curArmAngles = copy.deepcopy(liveArmAngles)
        dhTable = ik.createDHTable(curArmAngles)
        prevTargetTransform = ik.calculateTransformToLink(dhTable, 6)

        [newRoll, newPitch, newYaw] = ik.calculateRotationAngles(prevTargetTransform)
        newTargetValues = [newRoll, newPitch, newYaw, [prevTargetTransform[0][3], prevTargetTransform[1][3], prevTargetTransform[2][3]]]
        prevTargetValues = newTargetValues

        updateDesiredEETransformation(newTargetValues)
        updateDesiredArmSimulation(curArmAngles)        

class IKMode(ScriptState):
    def __init__(self) -> None:
        self.overriding = False


    def main() -> None:
        global curArmAngles
        global prevTargetValues
        global prevTargetTransform
        global goToPosValues

        publishNewAngles(curArmAngles)
        updateDesiredEETransformation(prevTargetValues)
        updateDesiredArmSimulation(curArmAngles)

        if goToPosValues[0]: # if the service flag to go to a saved position has been called
            goToPosValues[0] = False
            curArmAngles = goToPosValues[1]

            dhTable = ik.createDHTable(curArmAngles)
            prevTargetTransform = ik.calculateTransformToLink(dhTable, 6)

            [newRoll, newPitch, newYaw] = ik.calculateRotationAngles(prevTargetTransform)
            newTargetValues = [newRoll, newPitch, newYaw, [prevTargetTransform[0][3], prevTargetTransform[1][3], prevTargetTransform[2][3]]]
            prevTargetValues = newTargetValues
            
            updateDesiredEETransformation(newTargetValues)
            updateDesiredArmSimulation(curArmAngles)
    
    def onJoystickUpdate(isButtonPressed:dict, joystickAxesStatus:dict) -> None:
        global curArmAngles
        global prevTargetTransform
        global prevTargetValues
        global movementSpeed
        global isMovementNormalized

        try:
            dhTable = ik.createDHTable(curArmAngles)

            targetEEPos = controlEEPosition(isButtonPressed, joystickAxesStatus, prevTargetValues, 
                                        prevTargetTransform)
            
            targetAngles = ik.inverseKinematics(dhTable, targetEEPos) 

            maxIterations = 10
            iteration = 0
            originalSpeed = movementSpeed

            while isMovementNormalized and (iteration < maxIterations):
                if not limitAngleSpeed(targetAngles, curArmAngles):
                    break
                iteration += 1
                movementSpeed = movementSpeed*0.5

                targetEEPos = controlEEPosition(isButtonPressed, joystickAxesStatus, prevTargetValues, 
                                        prevTargetTransform)
                targetAngles = ik.inverseKinematics(dhTable, targetEEPos)

            movementSpeed = originalSpeed

            targetAngles.append(controlGripperAngle(isButtonPressed, curArmAngles))

            curArmAngles = targetAngles
            prevTargetTransform = targetEEPos
            prevTargetValues = newTargetValues
        except ik.CannotReachTransform:
            print("Cannot reach outside of arm workspace")
        except Exception as ex:
            print(ex)

    def controlEEPosition(self, prevTargetValues):
        global movementSpeed

        self.positionScale = 10*movementSpeed
        self.angleScale = 0.1*movementSpeed
       
        self.newTarget = copy.deepcopy(prevTargetValues)

        self.newRoll = self.newTarget[0]
        self.newPitch = self.newTarget[1]
        self.newYaw = self.newTarget[2]
    
        self.newX = self.newTarget[3][0]
        self.newY = self.newTarget[3][1]
        self.newZ = self.newTarget[3][2]

    def updateDesiredEEtransform(self):
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

        if self.overriding:
            return tempTarget

        referenceLink = "base_link" # link to base end effector transform off of

        sim.displayEndEffectorTransform(tempTarget, referenceLink) # display target transform

class DefaultIK(IKMode):
    def controlEEPosition(self, isButtonPressed, joystickAxis, prevTargetValues, prevTargetTransform):
        super().controlEEPosition(prevTargetValues)

        # control x, y, z
        if joystickAxis["L-Down"] != 0:
            self.newX -= joystickAxis["L-Down"]*self.positionScale
        if joystickAxis["L-Right"] != 0:
            self.newY -= joystickAxis["L-Right"]*self.positionScale
        if joystickAxis["L2"] != 0:
            self.newZ -= joystickAxis["L2"]*self.positionScale
        if joystickAxis["R2"] != 0:
            self.newZ += joystickAxis["R2"]*self.positionScale
        
        # control roll, pitch, yaw
        if joystickAxis["R-Right"] != 0:
            self.newYaw -= joystickAxis["R-Right"]*self.angleScale
        # if joystickAxis["R-Down"] != 0:
        #     self.newPitch -= joystickAxis["R-Down"]*scale2
        if isButtonPressed["DOWN"]:
            self.newPitch -= self.angleScale
        elif isButtonPressed["UP"]:
            self.newPitch += self.angleScale
        if isButtonPressed["L1"]:
            self.newRoll -= self.angleScale
        elif isButtonPressed["R1"]:
            self.newRoll += self.angleScale

        self.newTarget = [self.newRoll, self.newPitch, self.newYaw, [self.newX, self.newY, self.newZ]]
        # print("x: ", self.newX, " y: ", newY, " z: ", newZ, " roll: ", newRoll, " pitch: ", newPitch, " yaw: ", newYaw)
        newTargetValues = self.newTarget
        
        return ik.createEndEffectorTransform(self.newRoll, self.newPitch, self.newYaw, self.newTarget[3])

    def updateDesiredEETrasformtion(self):
        self.overriding = True
        tempTarget = super().updateDesiredEETrasformtion()
        tempTarget[1] = 0
        sim.displayEndEffectorTransform(tempTarget, "base_link") # display target transform

class GlobalIK(IKMode):
    def controlEEPosition(self, isButtonPressed, joystickAxis, prevTargetValues, prevTargetTransform):
        super().controlEEPosition(prevTargetValues)

        self.newRoll = 0
        self.newPitch = 0
        self.newYaw = 0

        if joystickAxis["L-Down"] != 0:
            self.newX -= joystickAxis["L-Down"]*self.positionScale
        if joystickAxis["L-Right"] != 0:
            self.newY -= joystickAxis["L-Right"]*self.positionScale
        if joystickAxis["L2"] != 0:
            self.newZ -= joystickAxis["L2"]*self.positionScale
        if joystickAxis["R2"] != 0:
            self.newZ += joystickAxis["R2"]*self.positionScale
        # control roll, pitch, yaw
        if joystickAxis["R-Right"] != 0:
            self.newRoll += joystickAxis["R-Right"]*self.angleScale
        if joystickAxis["R-Down"] != 0:
            self.newPitch += joystickAxis["R-Down"]*self.angleScale
        # if isButtonPressed["UP"]:
        #     self.newPitch = -self.angleScale
        # elif isButtonPressed["DOWN"]:
        #     self.newPitch = self.angleScale
        if isButtonPressed["L1"]:
            self.newYaw = -self.angleScale
        elif isButtonPressed["R1"]:
            self.newYaw = self.angleScale
        
        prevRotation = prevTargetTransform[:3, :3]
        
        newRotation = np.dot(ik.createRotationMatrix(self.newRoll, self.newPitch, self.newYaw, "ypr"), prevRotation)
        newTransformation = np.block([
                                        [newRotation, np.array([[self.newX, self.newY, self.newZ]]).transpose()],
                                        [0, 0, 0, 1]
                                        ])
        [correctedRoll, correctedPitch, correctedYaw] = ik.calculateRotationAngles(newTransformation)

        newTargetValues = [correctedRoll, correctedPitch, correctedYaw, [self.newX, self.newY, self.newZ]]
        # print("x: ", newX, " y: ", newY, " z: ", newZ, " roll: ", correctedRoll, " pitch: ", correctedPitch, " yaw: ", correctedYaw)
        
        return newTransformation
    
    def updateDesiredEETrasformtion(self):
        self.overriding = True
        tempTarget = super().updateDesiredEETrasformtion()
        tempTarget[:3] = [0, 0, 0]
        sim.displayEndEffectorTransform(tempTarget, "base_link") # display target transform

class RotRelativeIK(IKMode):
    def controlEEPosition(self, isButtonPressed, joystickAxis, prevTargetValues, prevTargetTransform):
        super().controlEEPosition(prevTargetValues)

        self.newRoll = 0
        self.newPitch = 0
        self.newYaw = 0

        if joystickAxis["L-Down"] != 0:
            self.newX -= joystickAxis["L-Down"]*self.positionScale
        if joystickAxis["L-Right"] != 0:
            self.newY -= joystickAxis["L-Right"]*self.positionScale
        if joystickAxis["L2"] != 0:
            self.newZ -= joystickAxis["L2"]*self.positionScale
        if joystickAxis["R2"] != 0:
            self.newZ += joystickAxis["R2"]*self.positionScale
        # control roll, pitch, yaw
        if joystickAxis["R-Right"] != 0:
            self.newRoll = -joystickAxis["R-Right"]*self.angleScale
        if joystickAxis["R-Down"] != 0:
            self.newPitch = joystickAxis["R-Down"]*self.angleScale
        if isButtonPressed["L1"]:
            self.newYaw = -self.angleScale
        elif isButtonPressed["R1"]:
            self.newYaw = self.angleScale
        
        prevRotation = prevTargetTransform[:3, :3]
        
        newRotation = np.matmul(prevRotation, ik.createRotationMatrix(self.newRoll, self.newPitch, self.newYaw, "ypr"))
        newTransformation = np.block([
                                        [newRotation, np.array([[self.newX, self.newY, self.newZ]]).transpose()],
                                        [0, 0, 0, 1]
                                        ])
        [correctedRoll, correctedPitch, correctedYaw] = ik.calculateRotationAngles(newTransformation)

        newTargetValues = [correctedRoll, correctedPitch, correctedYaw, [self.newX, self.newY, self.newZ]]

        return newTransformation

    def updateDesiredEEtransform(self):
        tempTarget = [0, 0 , pi, [0, 0, 198*0.47/450]]
        sim.displayEndEffectorTransform(tempTarget, "Link_6")
        
class RelativeIK(IKMode):
    def controlEEPosition(self, isButtonPressed, joystickAxis, prevTargetValues, prevTargetTransform):
        global curArmAngles
        [newRoll, newPitch, newYaw, newX, newY, newZ] = [0, 0, 0, 0, 0, 0]
        
        if joystickAxis["L-Down"] != 0:
            newX = joystickAxis["L-Down"]*self.positionScale
        if joystickAxis["L-Right"] != 0:
            newY = -joystickAxis["L-Right"]*self.positionScale
        if joystickAxis["L2"] != 0:
            newZ = -joystickAxis["L2"]*self.positionScale
        if joystickAxis["R2"] != 0:
            newZ = joystickAxis["R2"]*self.positionScale
        # control roll, pitch, yaw
        if joystickAxis["R-Right"] != 0:
            newRoll = joystickAxis["R-Right"]*self.angleScale
        if joystickAxis["R-Down"] != 0:
            newPitch = joystickAxis["R-Down"]*self.angleScale
        if isButtonPressed["L1"]:
            newYaw = -self.angleScale
        elif isButtonPressed["R1"]:
            newYaw = self.angleScale
        
        t05 = ik.calculateTransformToLink(ik.createDHTable(curArmAngles), 5) # transform to link 5
        r05 = t05[:3, :3] # rotation matrix of third link
        relativeXYZ = np.matmul(r05, np.transpose([np.array([newX, newY, newZ ])]))
        targetXYZ = relativeXYZ+np.transpose([prevTargetTransform[:3, 3]])

        prevRotation = prevTargetTransform[:3, :3]
        newRotation = np.matmul(prevRotation, ik.createRotationMatrix(newRoll, newPitch, newYaw, "ypr"))

        newTransformation = np.block([
                                        [newRotation, targetXYZ],
                                        [0, 0, 0, 1]
                                        ])

        [correctedRoll, correctedPitch, correctedYaw] = ik.calculateRotationAngles(newTransformation)
        [correctedX, correctedY, correctedZ] = [newTransformation[0][3], newTransformation[1][3], newTransformation[2][3]]
        newTargetValues = [correctedRoll, correctedPitch, correctedYaw, [correctedX, correctedY, correctedZ]]

        return newTransformation

    def updateDesiredEEtransform(self):
        tempTarget = [0, 0 , pi, [0, 0, 198*0.47/450]]
        sim.displayEndEffectorTransform(tempTarget, "Link_6")

class CamRelativeIK(IKMode):
    def controlEEPosition(self, isButtonPressed, joystickAxis, prevTargetValues, prevTargetTransform):
        pass

# Joystick Controller
def getJoystickButtons(tempButton:list) -> dict: # setting up the buttons
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
    
    buttons = {"X": 0, "CIRCLE": 0, "TRIANGLE": 0, "SQUARE": 0 , "L1": 0, "R1": 0, "L2": 0, "R2": 0, "SHARE": 0, "OPTIONS": 0, "PLAY_STATION": 0, 
        "L3": 0, "R3": 0,"UP": 0, "DOWN": 0, "LEFT": 0, "RIGHT": 0 } # 1 is pressed, 0 is not pressed, -1 is just released
 
    for i in range(0, tempButton.__len__()):
        button = tempButton[i]
        
        if buttonsPressed[BUTTON_NAMES[i]] == True and button == 0: # button just released
            button = 0#-1
        if buttonsPressed[BUTTON_NAMES[i]] == False and button == 1: # button just pressed
            button = 2

        if button < 1:
            buttonsPressed[BUTTON_NAMES[i]] = False
        else:
            buttonsPressed[BUTTON_NAMES[i]] = True
    
        buttons[BUTTON_NAMES[i]] = button
 
        
    return buttons

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
    global movementSpeed

    newTarget = copy.deepcopy(prevTargetValues)
    
    positionScale = 10*movementSpeed
    angleScale = 0.1*movementSpeed

    newRoll = newTarget[0]
    newPitch = newTarget[1]
    newYaw = newTarget[2]
    
    newX = newTarget[3][0]
    newY = newTarget[3][1]
    newZ = newTarget[3][2]

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
    global movementSpeed

    gripperAngle = curArmAngles[6]
    # Tune amount that angle is changed with each cycle
    angleIncrement = 0.1*movementSpeed

    # If both buttons are pressed at the same time, angle will not change
    if isButtonPressed["CIRCLE"] == 1 and isButtonPressed["X"] != 1:
        gripperAngle -= angleIncrement

    if isButtonPressed["X"] == 1 and isButtonPressed["CIRCLE"] != 1:
        gripperAngle += angleIncrement

    # if gripperAngle < -0.04:
    #     gripperAngle = -0.04
    # if gripperAngle > 0:
    #     gripperAngle = 0

    return gripperAngle

def limitAngleSpeed(newArmAngles, curArmAngles):
    ''' Limits the rate at which the end effector targert changes

    Helps arm joints move in sync (ish)

    '''

    jointSpeeds = [0.005, 0.003, 0.01, 0.075, 0.075, 0.0375, 0.04]
    angleDelta = list(np.subtract(np.array(newArmAngles), np.array(curArmAngles)))
    # slowedDownAngles = copy.deepcopy(newArmAngles)

    for i in range(7):
        if abs(angleDelta[i]) > jointSpeeds[i]:
            # exit, which tells ik to half distance of delta vector and try again until all joints can move there within the time frame
            return False

    return True

def incrementTargetAngles(newArmAngles, curArmAngles):
    ''' Increments the current angle to the desired angle
    
    '''
    jointSpeeds = [0.005, 0.003, 0.01, 0.075, 0.075, 0.0375, 0.04]
    angleDelta = list(np.subtract(np.array(newArmAngles), np.array(curArmAngles)))
    slowedDownAngles = copy.deepcopy(newArmAngles)

    try:
        for i in range(7):
            if abs(np.rad2deg(angleDelta[i])) > jointSpeeds[i]:
                if angleDelta[i] > 0:
                    slowedDownAngles[i] = curArmAngles[i] + np.deg2rad(jointSpeeds[i])
                else:
                    slowedDownAngles[i] = curArmAngles[i] - np.deg2rad(jointSpeeds[i])
    except Exception as ex:
        print("deg2rad conversion in incrementAngles")
    print(newArmAngles)
    print(curArmAngles)
    print(np.rad2deg(slowedDownAngles))
    return slowedDownAngles

# ROS Stuff

def savePosition(posName):
    ''' Service function that saves the current angles the arm is in

    Uses global variable curArmAngles and saves angles to file. Asks for name
    of position before saving (preferable a GUI).

    Paramters
    ---------
    posName
        the input message of SaveArmPos.srv, or the name to give the position
    
    Returns
    '''

    # (Placeholders for angles 1-5)

    # positionName = input("Enter Position Name: ")
    try:
        global curArmAngles

        positionName = posName.positionName
        print(positionName)

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

        with open('arm_positions.json','r+') as file:
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
        
        return SaveArmPosResponse(True)
    except Exception as ex:
        print("An error has occured")
        print(ex)
        return SaveArmPosResponse(False)

def goToPosition(posName):
    ''' Pulls up GUI with options of saved joint angles.

    Paramters
    ---------
    posName
        the input data message from GoToArmPos.srv, or the name of the saved position

    Returns
    -------
    service response
        has the service been completed sucessfully
    '''
    try:
        global scriptMode
        global goToPosValues

        # retrievePos = input("Enter Position Name to Retrieve: ")
        retrievePos = posName.positionName
        tempAngles = [0, 0, 0, 0, 0, 0, 0]

        with open('arm_positions.json','r') as file:
            found = False
            savedPos = json.load(file)
            # Checks to see if position names match request
            for x in range(len(savedPos["position_names"])):
                if savedPos["position_names"][x]["title"] == retrievePos: # if found, changes current angles to requested position
                    found = True
                    tempAngles[0] = savedPos["position_names"][x]["angle0"]
                    tempAngles[1] = savedPos["position_names"][x]["angle1"]
                    tempAngles[2] = savedPos["position_names"][x]["angle2"]
                    tempAngles[3] = savedPos["position_names"][x]["angle3"]
                    tempAngles[4] = savedPos["position_names"][x]["angle4"]
                    tempAngles[5] = savedPos["position_names"][x]["angle5"]
                    tempAngles[6] = savedPos["position_names"][x]["gripperAngle"]

            if not found:
                print("Position name '"+retrievePos+"' not found") # if not found, prints message
                return GoToArmPosResponse(False)

            goToPosValues[0] = True # sets the goTO arm position value true a
            goToPosValues[1] = tempAngles
            return GoToArmPosResponse(True)
    except Exception as ex:
        print("The following error has occured: ")
        print(ex)
        return GoToArmPosResponse(False)

def updateAngleCorrections(corrections):
    ''' Service function that updates the angle corrections for published IK values
    
    Paramters
    ---------
    corrections
        the input data message from Corrections.srv

    Returns
    -------
    service response
        has the service been completed sucessfully
    '''
    try:
        global angleCorrections

        angleCorrections = [corrections.correction1, corrections.correction2, corrections.correction3, corrections.correction4,
                            corrections.correction5, corrections.correction6, corrections.correction7]
        return CorrectionsResponse(True)
    except Exception as ex:
        print("The following error has occured")
        print(ex)
        return CorrectionsResponse(False)

def publishNewAngles(newJointAngles):
    ''' Publishes the new angles to /arm_target_angles

    Publishes a list 32 bit floats.

    Parameters
    ---------- 
    newJointAngles
        an array of the new angles (7 elements)

    '''
    global angleCorrections
    global savedCanAngles

    ikAngles = Float32MultiArray()
    adjustedAngles = [0, 0, 0, 0, 0, 0, 0]
    for i in range(7):
        adjustedAngles[i] = newJointAngles[i] - angleCorrections[i] - LIMIT_SWITCH_ANGLES[i] + savedCanAngles[i]
        adjustedAngles[i] = np.rad2deg(adjustedAngles[i])

    adjustedAngles[0] = -adjustedAngles[0]
    adjustedAngles[1] = -adjustedAngles[1]
    adjustedAngles[5] = -adjustedAngles[5]

    temp = adjustedAngles[5]
    adjustedAngles[5] = adjustedAngles[4]
    adjustedAngles[4] = temp

    ikAngles.data = adjustedAngles
    armAngles.publish(ikAngles)

def updateDesiredArmSimulation(armAngles):
    ''' Updates RViz URDF visulaztion of desired arm angles

    Paramters
    ---------
    armAngles
        contains a list 32 bit floats that correspond to joint angles
    '''
    global jointPublisher
    global gazeboPublisher
    global liveArmAngles

    tempAngles = copy.deepcopy(armAngles)
    tempAngles.append(tempAngles[6]) # make gripper angles equal
    tempAngles.append(tempAngles[6]) # make gripper angles equal

    tempAngles2 = copy.deepcopy(liveArmAngles)
    tempAngles2.append(tempAngles[6]) # make gripper angles equal
    tempAngles2.append(tempAngles[6]) # make gripper angles equal
    if gazebo_on:
        curArmAngles[2] = curArmAngles[2] - pi/2 # adjustment for third joint (shifted 90 degrees so it won't collide with ground on startup)
        # curArmAngles[6] = -curArmAngles[6]
        print(tempAngles)
        sim.moveInGazebo(gazeboPublisher, tempAngles)
    else:
        anglesToDisplay =  []
        
        for i in range(tempAngles2.__len__()):
            anglesToDisplay.append(tempAngles2[i])
        for i in range(tempAngles.__len__()):
            anglesToDisplay.append(tempAngles[i])
        # print(anglesToDisplay)
        sim.runNewJointState2(jointPublisher, tempAngles)

def updateLiveSim():
    global liveArmAngles
    global savedCanAngles
    global curArmAngles
    
    tempAngles = copy.deepcopy(liveArmAngles)#copy.deepcopy(list(np.subtract(np.array(liveArmAngles),np.array(savedCanAngles))))
    # tempAngles = copy.deepcopy(liveArmAngles)
    tempAngles.append(tempAngles[6]) # make gripper angles equal
    tempAngles.append(tempAngles[6]) # make gripper angles equal

    tempAngles[0] = tempAngles[0]
    tempAngles[1] = tempAngles[1]
    tempAngles[5] = -tempAngles[5]

    sim.runNewJointState3(jointPublisher, tempAngles)

def updateLiveArmSimulation(data):
    ''' Updates RViz URDF visulaztion based on IRL arm angles

    Should be used as the callback function of the real angle subscriber.

    Paramters
    ---------
    data
        data.data contains a list 32 bit floats that correspond to joint angles
    '''
    global liveArmAngles
    global savedCanAngles

    tempList = list(data.data)

    for i in range(7):
        tempList[i] = np.deg2rad(tempList[i])

    tempList[0] = -tempList[0]
    tempList[1] = -tempList[1]
    temp = tempList[5]
    tempList[5] = tempList[4]
    tempList[4] = temp
    # tempList[5] = -tempList[5]
    tempList[6] = -tempList[6]
    tempAngles = copy.deepcopy(list(np.subtract(np.array(tempList),np.array(savedCanAngles))))

    liveArmAngles = tempAngles
    # print(liveArmAngles)
    # tempAngles = copy.deepcopy(list(np.subtract(np.array(liveArmAngles),np.array(savedCanAngles))))
    # liveArmAngles = copy.deepcopy(tempAngles)
    # tempAngles.append(tempAngles[6]) # make gripper angles equal
    # tempAngles.append(tempAngles[6]) # make gripper angles equal

    # tempAngles[0] = -tempAngles[0]
    # tempAngles[1] = -tempAngles[1]
    # tempAngles[5] = -tempAngles[5]

    # sim.runNewJointState3(jointPublisher, tempAngles)
    # updateLiveSim()

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
        tempTarget = [0, 0 , pi, [0, 0, 198*0.47/450]]
        referenceLink = "Link_6"
    
    sim.displayEndEffectorTransform(tempTarget, referenceLink) # display target transform

def updateState(data):
    ''' Callback function for arm_state rostopic

    If a state has been changed enter into forward kinematics mode
    to copy the current arm angles.
    
    '''
    global scriptMode
    global ikEntered
    global savedCanAngles
    global curArmAngles
    global goToPosValues
    global prevTargetTransform
    global prevTargetValues
    global liveArmAngles


    if data.data != "IK":
        scriptMode = Mode.FORWARD_KIN
        ikEntered = False
    else:
    #     scriptMode = scriptMode
        if not ikEntered:
            savedCanAngles = copy.deepcopy(liveArmAngles) 
            scriptMode = Mode.DEFAULT_IK
            liveArmAngles = [0, 0, 0, 0, 0, 0, 0]
            curArmAngles = [0, 0, 0, 0, 0, 0, 0]

            dhTable = ik.createDHTable(curArmAngles)
            prevTargetTransform = ik.calculateTransformToLink(dhTable, 6)

            [newRoll, newPitch, newYaw] = ik.calculateRotationAngles(prevTargetTransform)
            newTargetValues = [newRoll, newPitch, newYaw, [prevTargetTransform[0][3], prevTargetTransform[1][3], prevTargetTransform[2][3]]]
            prevTargetValues = newTargetValues
            
            updateDesiredEETransformation(newTargetValues, scriptMode)
            updateDesiredArmSimulation(curArmAngles)
        ikEntered = True

def updateController(data):
    ''' Callback function for the arm_inputs topic

    '''

    global scriptMode
    global movementSpeed
    global isMovementNormalized

    buttonsPressed = [data.x, data.o, data.triangle, 0, data.l1, data.r1, data.l2, data.r2, data.share, data.options, 0, 0, 0, 0, 0, 0, 0]
    # isButtonPressed = {"X": data.x, "CIRCLE": data.o, "TRIANGLE": data.triagle, 
    #                         "SQUARE": 0, "L1": data.l1, "R1": data.r1, "L2": data.l2, 
    #                         "R2": data.r2, "SHARE": data.share, "OPTIONS": data.options, 
    #                         "PLAY_STATION": 0, "L3": 0, "R3": 0, "UP": 0, "DOWN": 0, 
    #                         "LEFT": 0, "RIGHT": 0} 
    isButtonPressed = getJoystickButtons(buttonsPressed)
    # print(isButtonPressed)
    joystickAxesStatus = {"L-Right": -data.l_horizontal, "L-Down": -data.l_vertical, "L2": data.l2, 
                          "R-Right": -data.r_horizontal, "R-Down": -data.r_vertical, "R2": data.r2}

    if isButtonPressed["TRIANGLE"] == 2: # changes mode when button is released
        if (scriptMode.value + 1) > len(Mode):
            scriptMode = list(Mode)[0]
        else:
            scriptMode = list(Mode)[scriptMode.value]
        print(scriptMode.name)

    if isButtonPressed["OPTIONS"] == 2:
        movementSpeed += 0.1/NODE_RATE
    if isButtonPressed["SHARE"] == 2:
        movementSpeed -= 0.1/NODE_RATE
        if movementSpeed < 0: # don't let movement speed go into negatives
            movementSpeed = 0

    if isButtonPressed["R3"] == 2:
        isMovementNormalized = not isMovementNormalized
        print("Normalized Movement: ", isMovementNormalized)

# Program Control

def main():
    ''' Function that runs the program
    
    Combines all code to takes Joystick values, chooses script mode, changes arm movement speed,
    changes EE transform, solves IK, publishes joints, and updates RViz.
    '''

    global curArmAngles
    global liveArmAngles
    global scriptMode
    global prevTargetTransform
    global prevTargetValues
    global newTargetValues
    global movementSpeed
    global goToPosValues

    updateLiveSim()

# Main Area

if __name__ == "__main__":
    angleCorrections = [0, 0, 0, 0, 0, 0, 0]
    curArmAngles = [0, 0, 0, 0, 0, 0, 0]
    liveArmAngles = [0, 0, 0, 0, 0, 0, 0]
    prevTargetValues = [0, 0, 0, [250, 0, 450]] # start position
    newTargetValues = prevTargetValues
    prevTargetTransform = ik.createEndEffectorTransform(prevTargetValues[0], prevTargetValues[1],
                                                        prevTargetValues[2], prevTargetValues[3])
    goToPosValues = [False, [0, 0, 0, 0, 0, 0, 0]]
    savedCanAngles = [0, 0, 0, 0, 0, 0, 0]

    # try:
    scriptMode = Mode.FORWARD_KIN
    print(scriptMode.name)

    rospy.init_node("arm_ik_and_viz")
    gazebo_on = rospy.get_param("/gazebo_on")
    rate = rospy.Rate(NODE_RATE) # run at 10Hz

    armAngles = rospy.Publisher("arm_goal_pos", Float32MultiArray, queue_size=10)
    rospy.Subscriber("arm_curr_pos", Float32MultiArray, updateLiveArmSimulation)
    rospy.Subscriber("arm_state", String, updateState)
    rospy.Subscriber("arm_inputs", ArmInputs, updateController)

    if gazebo_on:
        gazeboPublisher = sim.startGazeboJointControllers(9)
    else:
        jointPublisher = sim.startJointPublisher()

    # sets start target position equal to curArmAngles after they have been updated
    # while curArmAngles == [0, 0, 0, 0, 0, 0]:
    #     tempDHTable = ik.createDHTable(curArmAngles)
    #     prevTargetPos = ik.calculayteTransformToLink(tempDHTable, 5)

    correctionsService = rospy.Service('update_arm_corrections', Corrections, updateAngleCorrections)
    goToArmPosService = rospy.Service('move_arm_to', GoToArmPos, goToPosition)
    saveArmPosService = rospy.Service('save_arm_pos_as', SaveArmPos, savePosition)
    

    while not rospy.is_shutdown():
        try:  
            main()
        except Exception as ex:
            print(ex)
        rate.sleep()
