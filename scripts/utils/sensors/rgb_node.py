import serial
import rospy




ser = serial.Serial('/dev/ttyTHS0', baudrate=9600)  
print(ser.name)        
while True:
    ser.write(b'R')     
ser.close()   