import serial
import time

#arduino serial port name on linux
port_name = '/dev/ttyACM0'

#angles to toggle
angle_low = "63"
angle_high = "84"

#initializing serial port with baudrate 9600
arduino_port = serial.Serial(port_name, 9600)

def write_servo_high_angle():
    """
    (None) -> (None)
    Tell servo to go to high angle (angle_high)
    """
    #time.sleep(2)
    arduino_port.write((str(angle_high)).encode('utf-8'))
    #time.sleep(0.5)


def write_servo_low_angle():
    """
    (None) -> (None)
    Tell servo to go to low angle (angle_low)
    """
    #time.sleep(2)
    arduino_port.write((str(angle_low)).encode('utf-8'))
    #time.sleep(0.5)

def close_arduino_port():
    """
    (None) -> (None)
    Closes arduino port 
    """
    arduino_port.close()

# triggered = 0

# while 1:
#     # Getting angles from the remote controller
#     input_angles = MapJoystick(GetManualJoystickFinal.GetManualJoystick(), current_pos, speed_limit, t-time.time())
        
#     # Printing input angles from remote controller
#     print(input_angles)

#     # Going 63 degrees configuration
#     if input_angles[7] == 0 and triggered != 0: 
#         triggered = 0
#         write_servo_low_angle()
#         print("Set low")
    
#     # Going 84 degrees configuration
#     elif input_angles[7] == 1 and triggered != 1:
#         triggered = 1
#         write_servo_high_angle()
#         print("Set high")