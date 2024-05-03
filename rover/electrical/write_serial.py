import serial


ser = serial.Serial('/dev/ttyUSB0', 9600)
str = '001'
ser.write(str.encode())
ser.close()
