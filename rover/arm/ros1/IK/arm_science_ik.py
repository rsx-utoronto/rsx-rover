import numpy as np
from math import pi, atan2, sin, cos, sqrt, acos
from arm_class import Arm

class KinematicsMode():
    def __init__(self, name, kinCalcCallback, targetUpdateCallback):
        self.name = name
        self.kinCalcCallback = kinCalcCallback
        self.targetUpdateCallback = targetUpdateCallback

class SciArm(Arm):
    def __init__(self, numJoints:int, dhTable, offsets):
        super().__init__(numJoints, dhTable, offsets)
        self.modes = ["Forward", "Cyl"]
        # self.cylTarget = [0, 161+67, 80+348+110, 0] # [theta, r, z, alpha]


        self.cylTarget = [0, 0, 0, 0] # [theta, r, z, alpha]

    def move_to_position(self, position_name):
        # Move the arm to a specific position
        if position_name in self.sampling_positions:
            self.cylTarget = self.sampling_positions[position_name]
            return self.inverseKinematics()
        return False

    def execute_sampling_sequence(self):
        # Move to collection position
        if not self.move_to_position("collect"):
            return False
        
        # Move to intermediate position
        if not self.move_to_position("intermediate"):
            return False
        # Move to deposit position
        if not self.move_to_position("deposit"):
            return False
        
        return True 
            
    def setTarget(self, goalPos):
        self.cylTarget = goalPos

    def controlTarget(self, isButtonPressed, joystickStatus):
        if self.curMode == "Cyl":
            self.cylTarget[0] += joystickStatus["L-Right"]*self.CONTROL_SPEED # theta
            self.cylTarget[1] += joystickStatus["L-Down"]*self.CONTROL_SPEED
            self.cylTarget[3] += joystickStatus["R-Down"]*self.CONTROL_SPEED

            if isButtonPressed["R2"]:
                self.cylTarget[2] += self.CONTROL_SPEED*500
            if isButtonPressed["L2"]:
                self.cylTarget[2] -= self.CONTROL_SPEED*500

    def forwardKinematics(self):
        super().forwardKinematics()

    def inverseKinematics(self) -> bool:
        ''' Inverse
        
        Parameters
        ----------
        dhTable
            the DH table for the arm
        target
            [r, theta, z, alpha] - cylinderical coords with alpha being drill rotations


        Returns
        -------
        bool
            Returns True is taget is a valid point.
        '''
        d1 = self.dhTable[0][0]
        r2 = self.dhTable[1][2]
        r3 = self.dhTable[2][2]
        r4 = self.dhTable[3][2]
        d5 = self.dhTable[4][0]

        theta1 = self.cylTarget[0]

        r = self.cylTarget[1]
        z = self.cylTarget[2]
        alpha = self.cylTarget[3]

        link4Hyp = sqrt(r4**2 + d5**2)

        r_w = r - link4Hyp*cos(alpha)
        s = z  + link4Hyp*sin(alpha)
        print(f'{r_w} = {r} - {link4Hyp*cos(alpha)}')
        print(f'{s} = {z} + {link4Hyp*sin(alpha)}')

        # r_w = r
        # s = z

        cosTheta3Numerator = r_w**2 + (s - d1)**2 - r2**2 - r3**2
        # print(f'cosTheta3Numerator = {r_w**2} + {(s - d1)**2} - {r2**2} - {r3**2}')
        cosTheta3 = cosTheta3Numerator/(2*r2*r3)
        if abs(cosTheta3) > 1:
            return False 

        theta3 = atan2(sqrt(1 - cosTheta3**2), cosTheta3) 

        innerAngle = atan2(r3*sin(theta3), r2 + r3*cos(theta3))
        theta2 = atan2(s - d1, r_w) - innerAngle

        r_2 = r2*cos(theta2)
        z_2 = r2*sin(theta2) + d1
        print('r_2:', r_2, 'z_2:', z_2)
        h4 = sqrt((r-r_2)**2 + (z-z_2)**2)
        # print(f'{h4} = sqrt(({r}-{r_2})**2 + ({z}-{z_2}**2))')
        cosTheta4Numerator = h4**2 - r3**2 - link4Hyp**2
        cosTheta4 = cosTheta4Numerator/(2*r3*link4Hyp)
        theta4 = atan2(sqrt(1 - cosTheta4**2), cosTheta4)

        self.goalAngles = [theta1, theta2, theta3, -theta4]
        return True


