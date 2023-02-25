# Importing Libraries
# import serial
import time
import pygame
import numpy as np

running = True


def initializeJoystick():
    # robot operated with a joystick - uses pygame library
    pygame.init()
    global joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print('this works')
    print('Initialized joystick:', joystick.get_name())
    # print number of buttons
    print(joystick.get_numbuttons())


def setJoystickButtons():  # setting up the buttons
    pygame.event.pump()  # allow pygame to handle internal actions, keep everything current
    buttons = []

    for i in range(0, joystick.get_numbuttons()):
        button = joystick.get_button(i)
        buttons.append(button)
    return buttons


def getJoystickButtons():
    buttons = setJoystickButtons()
    return buttons


def getJoystickAnalogKeys():
    global analog_keys
    analog_keys = {0: 0, 1: 0, 2: 0, 3: 0, 4: -1, 5: -1}

    return analog_keys


def GetManualJoystick():
    ################################# CHECK PLAYER INPUT #################################
    joypos = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            ############### UPDATE SPRITE IF SPACE IS PRESSED #################################
            pass

        buttons = getJoystickButtons()
        


        # FOR BUTTON PRESSES -------------------------------------------------------
        if event.type == pygame.JOYBUTTONDOWN:
            if buttons[13]:
                joypos[13] = 1
            if buttons[14]:
                joypos[14] = 1
            if buttons[12]:
                joypos[12] = 1
            if buttons[11]:
                joypos[11] = 1
            if buttons[3]:
                joypos[7] = (1)                
            if buttons[0]:
                joypos[7] = (-1)                  
            if buttons[2]:
                joypos[9] = 1
            if buttons[1]:
                joypos[10] = 1
            if buttons[9]:
                joypos[5] = (-1)                 
            if buttons[10]:
                joypos[5] = (1)                  

        # FOR BUTTON RELEASES ---------------------------------------------------------
        if event.type == pygame.JOYBUTTONUP:
            if buttons[13] == 0:
                joypos[13] = 0
            if buttons[14] == 0:
                joypos[14] = 0
            if buttons[12] == 0:
                joypos[12] = 0
            if buttons[11] == 0:
                joypos[11] = 0
            if buttons[3] == 0:
                joypos[7] = 0                   
            if buttons[0] == 0:
                joypos[8] = 0
            if buttons[2] == 0:
                joypos[9] = 0
            if buttons[1] == 0:
                joypos[10] = 0
            if buttons[9] == 0:
                joypos[5] = 0                    
            if buttons[10] == 0:
                joypos[5] = 0                   
        # FOR ANALOG BUTTONS --------------------------------------------------------------------
        if event.type == pygame.JOYAXISMOTION:
            getJoystickAnalogKeys()
            analog_keys[event.axis] = event.value

            if (analog_keys[2] != 0):
                joypos[3] = -(analog_keys[2])              
            if (joypos[3] < .18 and joypos[3] > -.18):
                joypos[3] = 0

            if (analog_keys[3] != 0):
                joypos[2] = (analog_keys[3])
            if (joypos[2] < .18 and joypos[2] > -.18):
                joypos[2] = 0

            if (analog_keys[1] != 0):
                joypos[1] = (analog_keys[1])
            if (joypos[1] < .18 and joypos[1] > -.18):
                joypos[1] = 0

            if (analog_keys[0]):
                joypos[0] = -analog_keys[0]
            if (joypos[0] < .18 and joypos[0] > -.18):
                joypos[0] = 0

            if (analog_keys[5] != 1):
                joypos[4] = -((analog_keys[5] + 1) / (2))
            if (analog_keys[4] != -1):
                joypos[4] = ((analog_keys[4] + 1) / 2)
            if (joypos[4] < .1 and joypos[4] > -.1):
                joypos[4] = 0
            
            # if event.axis == 0:
            #     joypos[0] = (event.value + 1) * 90
            # elif event.axis == 1:
            #     joypos[1] = (event.value + 1) * 90
            # elif event.axis == 2:
            #     joypos[2] = (event.value + 1) * 90
            # elif event.axis == 3:
            #     joypos[3] = (event.value + 1) * 90
            # elif event.axis == 4:
            #     joypos[4] = (event.value + 1) * 90
            # elif event.axis == 5:
            #     joypos[5] = (event.value + 1) * 90
    return joypos


initializeJoystick()
setJoystickButtons()
getJoystickButtons()
getJoystickAnalogKeys()
#joypos = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

# while True:
#     print(getJoystickButtons())

# while (True):
#     joypos = GetManualJoystick()
#     print(joypos)
#     time.sleep(.1)
    

    #print(getJoystickButtons())



