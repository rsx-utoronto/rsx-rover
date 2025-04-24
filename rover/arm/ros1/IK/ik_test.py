from arm_science_ik import SciArm
from math import pi, atan2
import numpy as np

dhTable = [[79.7, 0, 0, pi/2],
            [0,    0, 367, 0],
            [0,    0, 195, 0],
            [0,    0, 67, pi/2],
            [92,   0, 0, 0]]
offsets = [0,
           -atan2(112.99, 348.08), 
           atan2(161, 110.7) + atan2(112.99, 348.08),
           (atan2(92, 67) + atan2(110.75, 161)),
           0]

arm = SciArm(5, dhTable, offsets)
arm.setTarget([0 , 550, -12.3, 0.941354])
status = arm.inverseKinematics()
print(status)
goalAngles = arm.getGoalAngles()
goalAngles.append(0)
print(arm.getOffsetGoalAngles())
print(goalAngles, type(goalAngles), "\n")

# arm.forwardKinematics()
# forwardKin = np.round(arm.calculateTransformToLink(3), 2)
# print(forwardKin, "\n")
goalAngles[3] = goalAngles[3] + 0.941354
arm.setCurAngles(goalAngles)
arm.forwardKinematics()
forwardKin = np.round(arm.calculateTransformToLink(5), 2)
print(forwardKin)
# print(np.round(arm.getDHTable(), 2))
arm.setCurAngles([0, 0, 0, 0, 0])
arm.forwardKinematics()
forwardKin = np.round(arm.calculateTransformToLink(5), 2)
print(forwardKin)
