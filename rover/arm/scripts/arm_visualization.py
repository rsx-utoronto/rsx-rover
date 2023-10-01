#!/usr/bin/env python3

import rospy
import arm_simulation_control as sim
from std_msgs.msg import String, Float32MultiArray
from numpy import deg2rad, subtract, array
from copy import deepcopy

class ArmVisualizationNode():
    ''' The node that visualizes the arm.

    The arm can be visalized both in RViz and gazebo. 
    When using with the real arm make sure to set the ros paramter
    /gazebo_on to false. If using the node with gazebo, set /gazebo_on
    to true so that node publishes joint messages to gazebo.
    
    '''

    def __init__(self):
        rospy.init_node("arm_viz")

        self.liveArmAngles = [0, 0, 0, 0, 0, 0, 0]
        self.savedCanAngles = [0, 0, 0, 0, 0, 0, 0]

        self.curMode = "Idle"
        self.gazebo_on =  rospy.get_param("/gazebo_on")
        self.ikEntered = False

        # ROS Topic Setup
        if self.gazebo_on:
            self.gazeboPublisher = sim.startGazeboJointControllers(9)
        else:
            self.jointPublisher = sim.startJointPublisher()

        rospy.Subscriber("arm_state", String, self.updateState)
        rospy.Subscriber("arm_goal_pos", Float32MultiArray, self.displayArmGoalPos)
        rospy.Subscriber("arm_curr_pos", Float32MultiArray, self.displayArmLivePos)
    
    def updateState(self, data):
        ''' Callbac function for the /arm_state topic

        Paramters
        ---------
        data
            data is a string message type. Use data.data to get the
            string that contains the name of the current mode.
        '''
        self.curMode = data.data

        if data.data == "IK":
            if not self.ikEntered:
                self.savedCanAngles = deepcopy(self.liveArmAngles)

            self.ikEntered = True

    def displayArmGoalPos(self, data):
        ''' Callback function for /arm_goal_pos
        
        Uses the goal position angles from /arm_goal_pos to display 
        the arm target

        Paramters
        ---------
        data
            A Float32MultiArray that contains the arm angles in degrees.
        '''
        tempAngles = list(deg2rad(list(data.data)))
        tempAngles = list(subtract(array(tempAngles), -1*array(self.savedCanAngles)))
        tempAngles.append(tempAngles[6]) # make gripper angles equal
        tempAngles.append(tempAngles[6]) # make gripper angles equal

        # unswap angles to match with joint order
        temp = tempAngles[5]
        tempAngles[5] = tempAngles[4]
        tempAngles[4] = temp

        # undo sign inversion for motors 
        tempAngles[0] = -tempAngles[0]
        tempAngles[1] = -tempAngles[1]
        tempAngles[5] = -tempAngles[5]

        if self.gazebo_on:
            sim.moveInGazebo(self.gazeboPublisher, tempAngles)
        else:
            sim.runNewDesiredJointState(self.jointPublisher, tempAngles)

    def displayArmLivePos(self, data):
        ''' Callback function for the /arm_curr_pos topic

        Use this callback function to update visualization of the 
        arms current position in RViz.

        Paramters
        ---------
        data
            data is a Float32MultiArray for the current arm positions.
            data.data contains 7 floats that describe the arms current
            position.
        '''
        tempList = list(deg2rad(list(data.data)))

        tempList[0] = -tempList[0]
        tempList[1] = -tempList[1]
        temp = tempList[5]
        tempList[5] = tempList[4]
        tempList[4] = temp
        # tempList[5] = -tempList[5]
        tempList[6] = tempList[6]
        tempAngles = list(subtract(array(tempList), array(self.savedCanAngles)))

        self.liveArmAngles = deepcopy(tempAngles)

        tempAngles.append(tempAngles[6])
        tempAngles.append(tempAngles[6])

        sim.runNewRealJointState(self.jointPublisher, tempAngles)

if __name__ == "__main__":

    try:
        ArmVizNode = ArmVisualizationNode()
        rospy.spin()
    except Exception as ex:
        print(ex) 
