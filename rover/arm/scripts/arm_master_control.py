#!/usr/bin/env python3

# import pygame
import numpy
import ik_library as ik
import arm_simulation_control as sim

# include controller stuff here

# Arm Control

def controlEEPosition(currentEETransform, isButtonPressed, joystickAxis):
    ''' Changes End Effector Position Matrix
    
    Takes Joystick Input and changes End Effector Transform Matrix accordingly

    Paramters
    ---------
    currentEETransform
        numpy matrix of current end effector transform

    isButtonPressed
        array that contains buttons pressed in order []
    
    joystickAxis
        array that contains floats that represent magnitude of X and Y directions that each joytstick is pointed in. 
        Also contains depth that triggers are pushed in.

    Returns
    -------
    numpy matrix
        numpy matrix of new end effector position 
    '''
    pass

def controlGrippperAngle(isButtonPressed)
    ''' Adjust the angle at which the gripper is open

    Moves all fingers by an equal amount. Focuses on L1 and L2.

    Paramters
    ---------
    isButtonPressed
        an array containing buttons pressed.
    
    '''
    pass

# ROS Stuff

def publishNewAngles(newJointAngles):
    ''' Publishes the new angles to /{Rostopic_Name}

    Parameters
    ---------- 
    newJointAngles
        an array of the new angles (6 elements)

    '''
    pass

def updateSimulation(data):
    ''' Updates RViz simulation based on real arm angles

    When not connected to real arm change input to ik results.

    Paramters
    ---------
    liveArmAngles
        an array of 8 floats containg angles from arm from /{ROS_Topic_Name}
    '''
    pass

'''

if __name__ == "__main__":

    try:
        while not rospy.is_shutdown():
            
    except Exception as ex:
        print("The Following Error has occured")
        print(ex)
'''