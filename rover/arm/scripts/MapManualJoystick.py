

import time
import pygame 



def SetJointSpeed(i,o,s,dt):
    return (s*i*dt)+o



def MapJoystick(I, O, S, dt):   #takes in the joystick input(I), the current position of the motors (O), the speed limits (S)

    O[0] = SetJointSpeed(I[___], O[0], S[0], dt)
    O[1] = SetJointSpeed(I[___], O[1], S[1], dt)
    O[2] = SetJointSpeed(I[___], O[2], S[2], dt)
    O[3] = SetJointSpeed(I[___], O[3], S[3], dt)
    O[4] = SetJointSpeed(I[___], O[4], S[4], dt)
    O[5] = SetJointSpeed(I[___], O[5], S[5], dt)
    O[6] = SetJointSpeed(I[___], O[6], S[6], dt)

    return O





def ManualDrive(GoalPos, SpeedLimit, dt):
    return MapJoystick(GetJoysticParams(), GoalPos, SpeedLimit, dt)


    