#!/usr/bin/env python3

import rospy
import random
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray

'''path[] saves incoming positions from path planning. path[0] is the first position the arm moves to.
when the rover starts moving to path[0], path[0] is deleted and replaced by the next in queue.

pathHistory[] saves all positions from path planning without deleting anything.'''

path = []
pathHistory = []

x = []
y = []

# used in initializeTrajectory to check if the arm is already trying to change positions
trajectoryRunning = False

# maximum average speed in deg/sec
speedMax = 30.0

simAngles = [0,0,0,0,0,0,0]
actualAngles = [0,0,0,0,0,0,0]

class quinticPolynomial:
    ''' Creates a quintic polynomial that determines the path of a joint angle.
    initialized with the following parameters:
    qI / qF - initial / final joint angle
    vI / vF - initial / final joint angular velocity
    aI / aF - initial / final joint angular acceleration
    tF - final time (time it takes to reach qF)'''

    def __init__(self, qI, qF, vI, vF, aI, aF, tF):
        if (qF - qI > 0):
            self.direction = 1
        else:
            self.direction = 0
        self.k = 1

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
        q = self.a0 + self.k*self.a1*time + (self.k**2)*self.a2*time**2 + \
            (self.k**3)*self.a3*time**3 + (self.k**4)*self.a4*time**4 + (self.k**5)*self.a5*time**5
        return q

    def calculateVel(self, time):
        dq = self.a1 + (2*self.a2*time) + (3*self.a3*time**2) + (4*self.a4*time**3) + (5*self.a5*time**4)
        return dq

    def calculateAccel(self, time):
        ddq = 2*self.a2 + 6*self.a3*time + 12*self.a4*time**2 + 20*self.a5*time**3
        return ddq

    def changeEndTime(self, k):
        self.k = k
        pass

class PIDcontroller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prevError = 0.0
        self.integral = 0.0
    
    def calculate(self, error, direction):
        if (direction == 0):
            error *= -1

        p = self.Kp * error

        self.integral = self.integral + error
        i = self.Ki * self.integral

        d = self.Kd * (error - self.prevError)
        self.prevError = error

        return (p + i + d)


def quinticTrajectory(posI, posF, time):
    ''' Uses class quinticPolynomial to calculate quintic trajectory for each joint angle.
    Updates arm_goal_pos at 10hz (which can be changed with the frequency variable)'''
    global pubArmAngles
    global rate
    global simAngles
    global actualAngles
    global x
    global y

    # matplot stuff:
    x = [None]
    y = [None]

    testTimeStart = rospy.get_time()

    frequency = 10
    period = 1 / frequency

    # weight of each pid output
    Kp = 0.01
    Ki = 0.01
    Kd = 0.000001

    publishPos = Float32MultiArray() # array of angles to publish

    t = 0 # time elapsed
    k = 1 # factor that quintic polynomials are stretched by

    q = [None] * 7 # array of angles calculated from polynomials
    pid = [None] * 7 # array of pid output for each angle

    for i in range(6):
        # by default, initial/final velocities/accelerations are 0
        q[i] = quinticPolynomial(posI[i], posF[i], 0, 0, 0, 0, time)
        # Kp Ki Kd can be adjusted for each individual angle, but for now they are uniform for all 7 angles
        pid[i] = PIDcontroller(Kp, Ki, Kd)

    # note that time is not an actual measurement of total time - it represents the total time of the most recently calculated polynomial
    while (t <= time):
        pid_out_largest = 0
        startTime = rospy.get_time()

        for i in range(6):
            pid_out = pid[i].calculate(simAngles[i] - actualAngles[i], q[i].direction)
            if (pid_out > pid_out_largest):
                pid_out_largest = pid_out

        t_factor = time / (time + pid_out_largest)
        k *= t_factor
        time = time + pid_out_largest

        t = t / t_factor

        print("end time is: ", time)
        print("curr time is: ", t)
        
        if (k == 0):
            k = 0.1 # if k is 0, error has accumulated (likely because arm stopped moving) --> this just prevents division by 0

        for i in range(6):
            q[i].changeEndTime(k)
            posI[i] = q[i].calculatePoint(t)

        x.append(posI[1])
        y.append(rospy.get_time() - testTimeStart)

        publishPos.data = posI
        pubArmAngles.publish(publishPos)

        endTime = rospy.get_time()
        rospy.sleep(period - (startTime - endTime))
        t = t + period

    testTimeEnd = rospy.get_time()


    print("Time elapsed: ", testTimeEnd - testTimeStart)
    return


def savePathPlanning(angles):
    global path
    global pathHistory
    path.append(list(angles.data))
    pathHistory.append(list(angles.data))
    return


def initializeTrajectory(curAngles):
    global trajectoryRunning
    global pubActualAngles #placeholder
    global speedMax
    global path
    global simAngles

    simAngles = list(curAngles.data)
    fakeAngles = [] #placeholder begins

    for i in range(6):
        fakeAngles.append(simAngles[i] + random.randint(0,2) - 1)

    fakeAnglesPosition = Float32MultiArray()
    fakeAnglesPosition.data = fakeAngles

    pubActualAngles.publish(fakeAnglesPosition) #placeholder ends

    if trajectoryRunning:
        print(list(curAngles.data))
        return
    
    if len(path) == 0:
        return
    else:
        trajectoryRunning = True

    posI = list(curAngles.data)
    posF = path.pop(0)
    distanceMax = 0

    for i in range(6):
        if (abs(posF[i] - posI[i]) > distanceMax):
            distanceMax = abs(posF[i] - posI[i])

    time = distanceMax / speedMax

    if (time != 0):
        quinticTrajectory(posI, posF, time)

    trajectoryRunning = False
    pass

def updateActualAngles(actualAngles_):
    global actualAngles

    actualAngles = list(actualAngles_.data)
    pass
    

def getAngles():
    global pubArmAngles
    global pubActualAngles #placeholder publisher
    global rate
    
    rospy.init_node('trajectoryPlanning')
    pubArmAngles = rospy.Publisher(
        "/arm_goal_pos", Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)

    pubActualAngles = rospy.Publisher("/ARM_ACTUAL_POSITION", Float32MultiArray, queue_size=10)

    rospy.Subscriber("/PATH_PLANNING", Float32MultiArray, savePathPlanning, queue_size=10)
    # PATH_PLANNING is a placeholder for the path planning node
    rospy.Subscriber("/arm_goal_pos", Float32MultiArray, initializeTrajectory, queue_size=1000000000)
    rospy.Subscriber("/ARM_ACTUAL_POSITION", Float32MultiArray, updateActualAngles, queue_size=10)
    # ARM_ACTUAL_POSITION is also a placeholder for sensor data
    rospy.spin()

if __name__ == '__main__':
    try:
        getAngles()
        plt.plot(y,x)
        plt.show()
    except rospy.ROSInterruptException:
        pass