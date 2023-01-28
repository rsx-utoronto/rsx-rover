#!/usr/bin/env python3
import rospy
import numpy as np
import math

# joystick stuff (re-use old code)


def createXYZRotationMatrix(roll: float, pitch: float, yaw: float):
    ''' Creates a rotation matrix based on XYZ euler angles (Row, Pitch, Yaw)

    Paramaters (euler angles)
    ----------
    roll (alpha)
        angle x-axis rotated (float)
    pitch (beta)
        angle y-axis rotates (float)
    yaw (gamma)
        angle z-axis rotates (float)

    Returns
    -------
    numpy matrix
        rotation matrix using XYZ euler angles
    '''

    # Convert input angles to radians

    roll *= math.pi / 180
    pitch *= math.pi / 180
    yaw *= math.pi / 180

    # Define sine and cosine of each angle to reduce # of calculations

    cRoll = math.cos(roll)
    sRoll = math.sin(roll)

    cPitch = math.cos(pitch)
    sPitch = math.sin(pitch)

    cYaw = math.cos(yaw)
    sYaw = math.sin(yaw)

    # Assemble 3x3 rotation matrix
    # Rotations are applied in the same order each time:
    # Roll -> Pitch -> Yaw
    # The result is the matrix below

    rotationMatrix = np.array([
                                [(cPitch * cYaw), (- cPitch * sYaw), (sPitch)] ,
                                [((cRoll * sYaw) + (cYaw * sRoll * sPitch)), ((cRoll * cYaw) - (sRoll * sPitch * sYaw)), (- cPitch * sRoll)],
                                [((sRoll * sYaw) - (cRoll * cYaw * sPitch)), ((cYaw * sRoll) + (cRoll * sPitch * sYaw)), (cRoll * cPitch)]
                                ])

    # Returns matrix

    return rotationMatrix


def createEndEffectorTransform(roll: float, pitch: float, yaw: float, position):
    ''' Creates the matrix that defines the transform of the end effector based on target

    Uses createXYZRotationMatrix() to define rotation portion of matrix.

    Paramaters
    ----------
    position
        list in format [x, y, z] that contains x, y, and z location matrix
    row
        angle x-axis rotates
    pitch
        angle y-axis roates
    yaw
        angle z-axis rotates
    
    Exceptions
    ----------
    outOfWorkspace
        when the target transformation is outside of the arms reach
    
    Returns
    -------
    numpy matrix
        transfromation matrix of target end effector position

    '''
    # define exception - outOfWorkspace

    class outOfWorkspace(exception):
        pass
    # Raised when end effector coordinates are out of arm's range

    # checks for out of workspace

    try:
        if True: # conditions for outOfWorkspace
            # turn position list into numpy array
            positionArray = np.array(position)

            # create 4x4 block matrix
            # | (rotation matrx) (position) |
            # |   0      0       0       1  |

            endEffectorTransform = np.block([
                                            [ createXYZRotationMatrix(roll, pitch, yaw) , np.transpose(positionArray) ] ,
                                            [ 0 , 0 , 0 , 1]
                                            ])

            return endEffectorTransform
        else:
            raise outOfWorkspace
    except outOfWorkspace:
        print("Coordinates are out of workspace")
        return None

def createTransformationMatrix(d: float, theta: float, r: float , alpha: float):
    ''' Creates a transform matrix based on dh-table paramters.

    Input is assumed to be relative to user defined origin (base_link, link_1, etc). Uses
    matrix 3.10 (p.g 79) from ECE470 textbook. 

    Paramters
    ---------
    d
        d value from dh-table
    theta
        theta value from dh-table
    r
        r value from dh-table
    alpha
        alpha value from dh-table

    Return
    ------
    numpy matrix
        the transfromation matrix based on the given paramters of the dh-table
    '''
    # convert theta and alpha to radians

    theta *= math.pi / 180
    alpha *= math.pi / 180

    # define cos and sin for theta and alpha

    cosTheta = math.cos(theta)
    sinTheta = math.sin(theta)

    cosAlpha = math.cos(alpha)
    sinAlpha = math.sin(alpha)

    DHTransformMatrix = np.array([
                                [ cosTheta , (-sinTheta * cosAlpha) , (sinTheta * sinAlpha) , (r * cosTheta)] ,
                                [ sinTheta , (cosTheta * cosAlpha) , (-cosTheta * sinAlpha) , (r * sinTheta)] ,
                                [ 0 , sinAlpha , cosAlpha , d] ,
                                [ 0 , 0 , 0 , 1 ]
                                ])

    return DHTransformationMatrix

def calculateTransformToLink(transforms, linkNumber):
    ''' Find the transform matrix to specified location

    Basically just multiplies all elements in transforms from 0 to linkNumber

    Paramters
    ---------
    transforms
        array of transform matrices for each joint/link
    
    linkNumber
        the number of the link you want the transform of (0 is base)

    Returns
    ------
    numpy matrix
        the multiplied matrix
    '''
    pass

def createDHTable(jointAngles):
    ''' Create DH Table for arm based on current position 

    *include detailed description here based on what you made the function do*
    Change return type - the " -> list" to match numpy matrix

    Paramaters
    ---------
    jointAngles
        array for changing angles (theta star of dh-table)

    Returns
    -------
    numpy matrix
        a matrix containing the dh table (numpy matrix isn't the name of the return variable, just what type of data is returned)
    '''
    pass

def inverseKinematics(dhTable, targetPos):
    ''' Calculates joint angles based on desired EE position and DH-Table Paramters

    Parameters
    ----------
    dhTable
        the DH-Table for the Arm
    
    targetPos
        a numpy matrix that defines the desired EE positions
    
    Returns
    -------
    array
        list of joint angles
    '''
    pass
