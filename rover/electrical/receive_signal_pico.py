#!/usr/bin/env python3

import serial

# Replace '/dev/ttyUSB0' with the correct serial port of your Pico
ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)

while True:
    
    data = ser.readline()  # Read one byte
    print(len(data))
    print(data.decode('utf-8').strip())

