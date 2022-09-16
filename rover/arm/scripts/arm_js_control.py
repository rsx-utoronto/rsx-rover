# Importing Libraries
import serial
import time
import pygame
import numpy as np

arduino = serial.Serial(port='COM3', baudrate=115200, timeout=.1)







    #arduino.write(bytes(command, 'utf-8'))

def initializeJoystick():
# robot operated with a joystick - uses pygame library
    pygame.init()
    global joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print('Initialized joystick: %s' % joystick.get_name())
    print(joystick.get_numbuttons())

def getJoystickButtons(): # setting up the buttons
    pygame.event.pump() # allow pygame to handle internal actions, keep everything current
    
    global buttonsPressed
    
    buttons = []
 
    for i in range(0, joystick.get_numbuttons()):
        button = joystick.get_button(i)
        buttons.append(button)
     
    return buttons


def read_arduino():
    while(arduino.in_waiting!=0):
        print(str(arduino.readline()))



wasButtonPressed = np.zeros(19)
initializeJoystick()
while True:
    read_arduino()

    isButtonPressed = getJoystickButtons()
    #print(isButtonPressed)
    #Bicep
    if isButtonPressed[9]:
        arduino.write("b100".encode("utf-8"))
    elif isButtonPressed[11]:
        arduino.write("b-100".encode("utf-8"))
    elif wasButtonPressed[9] != isButtonPressed[9] or isButtonPressed[11]!=wasButtonPressed[11]: # just released
        pass
        arduino.write("b0".encode("utf-8"))
    time.sleep(0.02)
    # tricep
    if isButtonPressed[8]:
        arduino.write("t100".encode("utf-8"))
    elif isButtonPressed[10]:
        arduino.write("t-100".encode("utf-8"))
    elif wasButtonPressed[8] != isButtonPressed[8] or isButtonPressed[10]!=wasButtonPressed[10]: # just released
        pass
        arduino.write("t0".encode("utf-8"))
    time.sleep(0.02)
    #Gripper
    if isButtonPressed[12]:
        arduino.write("g100".encode("utf-8"))
    elif isButtonPressed[14]:
        arduino.write("g-100".encode("utf-8"))
    elif wasButtonPressed[12] != isButtonPressed[12] or isButtonPressed[14]!=wasButtonPressed[14]: # just released
        pass
        arduino.write("g0".encode("utf-8"))
    time.sleep(.02)

    #base
    if isButtonPressed[2] and isButtonPressed[7]:
        arduino.write("h100".encode("utf-8"))
    elif isButtonPressed[2] and isButtonPressed[5]:
        arduino.write("h-100".encode("utf-8"))
    elif wasButtonPressed[2] != isButtonPressed[2] or  wasButtonPressed[5] != isButtonPressed[5] or  wasButtonPressed[7] != isButtonPressed[7]: # just released
        pass
        arduino.write("h0".encode("utf-8"))
    time.sleep(.02)


    wasButtonPressed = isButtonPressed
    
