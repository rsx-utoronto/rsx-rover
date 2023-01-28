#!/usr/bin/env python3
import rospy
import numpy as np
import math

# joystick stuff (re-use old code)


def createXYZRotationMatrix(roll: float, pitch: float, yaw: float) -> list:
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

    rotationMatrix = np.array([[(cPitch * cYaw), (- cPitch * sYaw), (sPitch)],
                               [((cRoll * sYaw) + (cYaw * sRoll * sPitch)), ((cRoll * cYaw) - (sRoll * sPitch * sYaw)), (- cPitch * sRoll)],
                               [((sRoll * sYaw) - (cRoll * cYaw * sPitch)), ((cYaw * sRoll) + (cRoll * sPitch * sYaw)), (cRoll * cPitch)]])

    # Returns matrix

    return rotationMatrix


def createEndEffectorTransform(roll: float, pitch: float, yaw: float, position) -> list:
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

    '''

    # turn position list into numpy array
    positionArray = np.array(position)

    # create 4x4 block matrix
    # | (rotation matrx) (position) |
    # |   0      0       0       1  |

    endEffectorTransform = np.block([
                                    [ createXYZRotationMatrix(roll, pitch, yaw) , np.transpose(positionArray) ] ,
                                    [ 0 , 0 , 0 , 1]
                                    ])

    '''

    Exceptions
    ----------
    outOfWorkspace
        when the target transformation is outside of the arms reach

    Returns
    -------
    numpy matrix
        transfromation matrix of target end effector position
    '''
    return endEffectorTransform


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
