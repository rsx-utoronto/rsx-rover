import serial
import time


class write_serial():
    def __init__(self, port):
        self.port = port
        self.ser = serial.Serial(port, 9600)
    def write(self, str):
        while True:
            self.ser.write(str.encode('utf-8'))
            time.sleep(0.1)
        ser.close()


# ser = serial.Serial('/dev/ttyACM0', 9600)
# str = 'a'
# # a, g, m
# # a is autonomy -> red
# # g is goal -> green flashing
# # m is manual -> blue
# while True:
#     # print(ser.read())

#     ser.write(str.encode('utf-8'))
#     # print to terminal 
    
#     # print(str.encode('utf-8'))
#     time.sleep(0.1)
# ser.close()
