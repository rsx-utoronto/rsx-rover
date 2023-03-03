

import time
import pygame 
import GetManualJoystickFinal
#import anglepos


joy_input = GetManualJoystickFinal.GetManualJoystick()

current_pos = [0,0,0,0,0,0,0,0]
speed_limit = [20000/100, 20000/100, 80000/100, 20000/100, 20000/20, 20000/20, 800000/20]                    # Gear Ratio are after the /
t = 0



def SetJointSpeed(joy_input, current_pos, speed_limit, dt):

    return (speed_limit*joy_input*dt)+ current_pos



def MapJoystick(joy_input, current_pos, speed_limit, dt):   #takes in the joystick input(I), the current position of the motors (O), the speed limits (S)

    current_pos[0] = SetJointSpeed(joy_input[0], current_pos[0], speed_limit[0], dt)
    current_pos[1] = SetJointSpeed(-joy_input[1], current_pos[1], speed_limit[1], dt)
    current_pos[2] = SetJointSpeed(joy_input[3], current_pos[2], speed_limit[2], dt)
    current_pos[3] = SetJointSpeed(joy_input[2], current_pos[3], speed_limit[3], dt)
    current_pos[4] = SetJointSpeed(joy_input[4], current_pos[4], speed_limit[4], dt)
    current_pos[5] = SetJointSpeed(joy_input[5], current_pos[5], speed_limit[5], dt)
    current_pos[6] = SetJointSpeed(joy_input[6], current_pos[6], speed_limit[6], dt)
    current_pos[7] = joy_input[7]
    


    return current_pos




#  def ManualDrive(GoalPos, SpeedLimit, dt):
#      return MapJoystick(GetJoysticParams(), GoalPos, SpeedLimit, dt)

# while (True):
    
#     pos = MapJoystick(GetManualJoystickFinal.GetManualJoystick(), current_pos, speed_limit, t-time.time())
#     t = time.time()
#     print(pos)
#     time.sleep(.01)

    
