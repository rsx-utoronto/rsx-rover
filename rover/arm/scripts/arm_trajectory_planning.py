#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

path = []
pathHistory = []
trajectoryRunning = False # used in initializeTrajectory to check if the arm is already trying to change positions


class quinticPolynomial:
    def __init__(self, qI, qF, vI, vF, aI, aF, tF):
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
        q = self.a0 + self.a1*time + self.a2*time**2 + \
            self.a3*time**3 + self.a4*time**4 + self.a5*time**5
        return q


def quinticTrajectory(posI, posF, time):
    t = 0
    q = [None] * 7
    armAngles = rospy.Publisher("arm_angles", Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)

    for i in range(7):
        # by default, initial/final velocities/accelerations are 0
        q[i] = quinticPolynomial(posI[i], posF[i], 0, 0, 0, 0, time)

    while (t <= time):
        startTime = rospy.get_time()

        for i in range(7):
            posI[i] = q[i].calculatePoint(t)

        endTime = rospy.get_time()  # need to fix the time stuff
        armAngles.publish(posI)
        rate.sleep()
        t = t + (endTime - startTime)

    for i in range(7):
        posI[i] = posF[i]

    pass


def savePathPlanning(angles):
    path.append(angles)
    pathHistory.append(angles)
    pass


def initializeTrajectory(curAngles):
    # how do I stop the function from running again while it runs quinticTrajectory?
    if trajectoryRunning:
        pass
    else:
        trajectoryRunning = True
        
    if len(path) == 0:
        pass

    time = 1 # needs a way of controlling time

    quinticTrajectory(curAngles, path[0], time)
    path.pop(0)

    trajectoryRunning = False
    pass


def getAngles():
    rospy.init_node('trajectoryPlanning', anonymous=True)
    rospy.Subscriber("PATH_PLANNING", Float32MultiArray, savePathPlanning)
    # PATH_PLANNING is a placeholder
    rospy.Subscriber("arm_angles", Float32MultiArray, initializeTrajectory)
    rospy.spin()

if __name__ == '__main__':
    try:
        getAngles()
    except rospy.ROSInterruptException:
        pass
