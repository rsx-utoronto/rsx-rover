
import rclpy
from rclpy.node import Node 
import serial 


ser = serial.Serial('/dev/ttyACM1')
ser.baudrate = 115200

print(ser.name)

while True: 
    ser.write(b'b')
ser.close()