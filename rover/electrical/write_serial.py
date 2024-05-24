import serial
import time


ser = serial.Serial('/dev/ttyACM0', 9600)
str = 'g'
while True:
    # print(ser.read())

    ser.write(str.encode('utf-8'))
    # print(str.encode('utf-8'))
    time.sleep(0.1)
ser.close()
