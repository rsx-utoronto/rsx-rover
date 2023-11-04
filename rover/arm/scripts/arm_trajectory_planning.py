#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

'''path[] saves incoming positions from path planning. path[0] is the first position the arm moves to.
when the rover starts moving to path[0], path[0] is deleted and replaced by the next in queue.

pathHistory[] saves all positions from path planning without deleting anything.'''

path = []
pathHistory = []
simAngles = np.array([0,0,0,0,0,0,0])
# used in initializeTrajectory to check if the arm is already trying to change positions
trajectoryRunning = False
speedMax = 10.0

actualAngles = np.array([0,0,0,0,0,0,0])
#new_angle is the difference between published angles and real angles from sensor data (set to 0 by default)


class quinticPolynomial:
    ''' Creates a quintic polynomial that determines the path of a joint angle.
    initialized with the following parameters:
    qI / qF - initial / final joint angle
    vI / vF - initial / final joint angular velocity
    aI / aF - initial / final joint angular acceleration
    tF - final time (time it takes to reach qF)'''

    def __init__(self, qI, qF, vI, vF, aI, aF, tF):
        #self.tF = tF
        if (tF == 0):
            self.a0 = qF
            self.a1 = 0
            self.a2 = 0
            self.a3 = 0
            self.a4 = 0
            self.a5 = 0
        else:
            self.a0 = qI
            self.a1 = vI
            self.a2 = aI / 2

            # needs fixing --> how to decide safest/most efficient path?

            A = np.array([[tF**3, tF**4, tF**5],
                        [3*tF**2, 4*tF**3, 5*tF**4],
                        [6*tF, 12*tF**2, 20*tF**3]])
            b = np.array([[qF - qI - vI*tF - aI*tF**2],
                        [vF - vI - aI*tF],
                        [aF - aI]])
        
            x = np.linalg.solve(A, b)

            self.a3 = x[0]
            self.a4 = x[1]
            self.a5 = x[2]
 

    def calculatePoint(self, time):
        ''' Uses polynomial to calculate desired angle at specified time'''
        q = self.a0 + self.a1*time + self.a2*time**2 + \
            self.a3*time**3 + self.a4*time**4 + self.a5*time**5
        return q

    def calculateSlope(self, time):
        dq = self.a1 + (2*self.a2*time) + (3*self.a3*time**2) + (4*self.a4*time*3) + (5*self.a5*time*4)
        return dq

def quinticTrajectory(posI, posF, time, bigIndex):
    ''' Uses class quinticPolynomial to calculate quintic trajectory for each joint angle.
    Updates arm_goal_pos at 10hz (which can be changed with the frequency variable)'''
    global pubArmAngles
    global rate

    frequency = 10
    period = 1 / frequency

    publishPos = Float32MultiArray()

    t = 0
    q = [None] * 7

    for i in range(6):
        # by default, initial/final velocities/accelerations are 0
        q[i] = quinticPolynomial(posI[i], posF[i], 0, 0, 0, 0, time)

    while (t <= time):
        startTime = rospy.get_time()

        for i in range(6):
            posI[i] = q[i].calculatePoint(t)

        publishPos.data = posI
        pubArmAngles.publish(publishPos)

        endTime = rospy.get_time()
        rospy.sleep(period - (startTime - endTime))
        t = t + period
        while not (np.all(np.around(simAngles, 1) == np.around(actualAngles, 1))):
            continue

    for i in range(6):
        posI[i] = posF[i]
        publishPos.data = posI
        pubArmAngles.publish(publishPos)
    return


def savePathPlanning(angles):
    global path
    global pathHistory
    path.append(list(angles.data))
    pathHistory.append(list(angles.data))
    #print("I RECEIVED AN ANGLE")
    #print("path[] = ", path)
    return


def initializeTrajectory(curAngles):
    global trajectoryRunning
    global speedMax
    global path
    global simAngles

    simAngles = np.array(list(curAngles.data))
    print("simAngles is: ", simAngles)

    #print("I TRIED RUNNING")
    if trajectoryRunning:
        print(list(curAngles.data))
        return
    if len(path) == 0:
        return
    else:
        #print("IM RUNNING")
        trajectoryRunning = True

    #testTimeStart = rospy.get_time()

    #print("angle received: ", curAngles.data)
    posI = list(curAngles.data)
    #print("Path before pop: ", path)
    posF = path.pop(0)
    #print("Path after pop: ", path)
    distanceMax = 0
    bigIndex = 0

    # time is determined by the joint which must travel the farthest
    for i in range(6):
        if (abs(posF[i] - posI[i]) > distanceMax):
            distanceMax = abs(posF[i] - posI[i])
            bigIndex = i
            #print("distanceMax was changed to ", distanceMax)

    time = distanceMax / speedMax

    #print("I'm going to ", posF, "\nfrom ", posI)
    quinticTrajectory(posI, posF, time, bigIndex)

    #testTimeEnd = rospy.get_time()

    #print("Ideal time: ", time , " Elapsed time: ", testTimeEnd - testTimeStart)
    trajectoryRunning = False
    pass

def calculateRealSpeed(actualAngles_):
    #global simAngles
    #global new_angle
    global actualAngles

    print(actualAngles_.data)
    actualAngles = np.array(list(actualAngles_.data))

    #new_angle = abs(list(actual_angles.data) - simAngles)
    pass
    

def getAngles():
    global pubArmAngles
    global rate
    
    rospy.init_node('trajectoryPlanning')
    pubArmAngles = rospy.Publisher(
        "/arm_goal_pos", Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)

    rospy.Subscriber("/PATH_PLANNING", Float32MultiArray, savePathPlanning)
    # PATH_PLANNING is a placeholder
    rospy.Subscriber("/arm_goal_pos", Float32MultiArray, initializeTrajectory, queue_size=10)
    rospy.Subscriber("/ARM_ACTUAL_POSITION", Float32MultiArray, calculateRealSpeed, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        getAngles()
    except rospy.ROSInterruptException:
        pass
