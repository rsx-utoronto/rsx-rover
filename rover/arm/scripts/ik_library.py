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

    return DHTransformMatrix

def calculateTransformToLink(dhTable, linkNumber):
    ''' Find the transform matrix to specified location '''
    transformToLink = np.eye(4,dtype=float)

    
    '''
    Basically just multiplies all element transforms from 0 to linkNumber. Uses createTransformMatrix().

    Paramters
    ---------
    dhTable
        dhTable of arm
    
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
        a matrix containing the dh table in [r, alpha, d, theta*] order
    '''

    DHTable = np.array([[0, math.pi/2, 1, jointAngles[0]],
                [1, 0, 0, jointAngles[1]],
                [0, math.pi/2, 0, jointAngles[2]],
                [0, -math.pi/2, 1, jointAngles[3]],
                [0, math.pi/2, 0, jointAngles[4]],
                [0, 0, 1, jointAngles[5]]] )
    return DHTable

def inverseKinematics(dhTable, targetPos):
    ''' Calculates joint angles based on desired EE position and DH-Table Paramters

    Step by step solution can be found pages 148 - 151 and 153-154 of ECE470 textbook.
    

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

    try:
        d1 = dhTable[0][2]
        a2 = dhTable[1][0]
        d4 = dhTable[3][2]
        d6 = dhTable[5][2]

        desiredWristRotation = np.array([targetPos[0][:3],targetPos[1][:3], targetPos[2][:3]])
        desiredWristOrigin = np.transpose(np.array([targetPos[0][3], targetPos[1][2], targetPos[2][3]]))
        
        # calculate wrist center
        wristCenter = desiredWristOrigin - d6 * desiredWristRotation * np.transpose(np.array([0, 0, 1]))
        
        wristCenterX = wristCenter[0]
        wristCenterY = wristCenter[1]
        wristCenterZ = wristCenter[2]

        theta1 = math.atan2(wristCenterY, wristCenterX)
        
        cosTheta3Numerator = wristCenterX**2 + wristCenterY**2 + (wristCenterZ-d1)**2 - a2**2 - d4**2
        cosTheta3 = cosTheta3Numerator/(2*a2*d4) 
        
        if abs(cosTheta3) > 1: # cos(x) cannot be greater than 1
            print("Can not reach transform")
            # return current joint angles
            return [dhTable[0][3], dhTable[1][3], dhTable[2][3], dhTable[3][3], dhTable[4][3], dhTable[5][3]]
            
        theta3 = math.atan2(math.sqrt(1 - cosTheta3**2), cosTheta3) # positive in front of square root assumes elbow up

        theta2 = math.atan2(wristCenterZ - d1, math.sqrt(wristCenterX**2 + wristCenterY**2))- math.atan2(d4*math.sin(theta3 - math.pi/2), a2 + d4*cosTheta3)
    
        t03 = calculateTransformToLink(createDHTable([theta1, theta2, theta3, 0, 0 ,0]), 3) # transform matrix from base to third link
        r03 = np.array([t03[0][:3], t03[1][:3], t03[2][:3]]) # rotation matrix of third link
        r36 = np.matmul(np.linalg.inv(r03), desiredWristOrigin ) # rotation matrix from third link to EE
        
        theta5 = math.atan2(math.sqrt(1-r36[2][2]**2), r36[2][2])

        theta4 = math.atan2(r36[1][2], r36[0][2])

        theta6 = math.atan2(r36[2][1], -r36[2][0])

        return [theta1, theta2, theta3, theta4, theta5, theta6]

    except Exception as ex:
        print("The following error occured: ")
        print(ex)
        # return current joint angles
        return [dhTable[0][3], dhTable[1][3], dhTable[2][3], dhTable[3][3], dhTable[4][3], dhTable[5][3]]
