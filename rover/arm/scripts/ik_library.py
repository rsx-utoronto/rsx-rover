#!/usr/bin/env python3
import rospy

# joystick stuff (re-use old code)

def createXYZRotationMatrix(roll, pitch, yaw) -> list:
    ''' Creates a rotation matrix based on XYZ euler angles (Row, Pitch, Yaw)

    Paramaters
    ----------
    roll
        angle x-axis rotated (double)
    pitch
        angle y-axis rotates (double)
    yaw
        angle z-axis rotates (double)
    
    Returns
    -------
    ### numpy matrix
        rotation matrix using XYZ euler angles  
    '''
    pass

def createEndEffectorTransform() -> list:
    ''' Creates the matrix that defines the transform of the end effector based on target

    Uses createXYZRotationMatrix to define rotation portion of matrix.

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

def createTransformationMatrix():
    ''' Creates a transformation

    '''
    pass

def createDHTable(jointAngles) -> list:
    ''' Create DH Table for arm based on current position 

    *include detailed description here based on what you made the function do*
    Change return type - the " -> list" to match numpy matrix

    Paramaters
    ---------
    jointAngles
        array for changing angles (theta star)

    Returns
    -------
    numpy matrix
        a matrix containing the dh table (numpy matrix isn't the name of the return variable, just what type of data is returned)
    '''
    pass

createDHTable()