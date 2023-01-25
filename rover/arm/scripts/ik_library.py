#!/usr/bin/env python3
import rospy
import numpy as np
import math

# joystick stuff (re-use old code)

def createXYZRotationMatrix(roll:float, pitch:float, yaw:float) -> list:

    ''' Creates a rotation matrix based on XYZ euler angles (Row, Pitch, Yaw)

    Paramaters
    ----------
    roll
        angle x-axis rotated (float) - 'alpha'
    pitch
        angle y-axis rotates (float) - 'beta'
    yaw
        angle z-axis rotates (float) - 'gamma'
    
    Returns
    -------
    numpy matrix
        rotation matrix using XYZ euler angles
    '''

    ''' Convert degrees to radians '''

    roll = roll * math.pi / 180
    pitch = pitch * math.pi / 180
    yaw = yaw * math.pi / 180

    ''' Define cosine and sine of each angle before entering into matrix to reduce # of calculations '''

    cRoll = math.cos(roll)
    sRoll = math.sin(roll)

    cPitch = math.cos(pitch)
    sPitch = math.sin(pitch)

    cYaw = math.cos(yaw)
    sYaw = math.sin(yaw)

    ''' Assemble matrix using multiplication of x, y, and z-axis rotation matrices '''

    rotationMatrix = np.array([ [(cPitch * cYaw) , (- cPitch * sYaw) , (sPitch)] ,
                                [((cRoll * sYaw) + (cYaw * sRoll * sPitch)) , ((cRoll * cYaw) - (sRoll * sPitch * sYaw)) , (- cPitch * sRoll)] ,
                                [((sRoll * sYaw) - (cRoll * cYaw * sPitch)) , ((cYaw * sRoll) + (cRoll * sPitch * sYaw)) , (cRoll * cPitch)] ])

    return rotationMatrix

def createEndEffectorTransform() -> list:
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
    pass

def createTransformationMatrix() -> list:
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
    pass

def createDHTable(jointAngles) -> list:
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

