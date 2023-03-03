import serial

#arduino serial port name on linux
port_name = '/dev/ttyACM0'

#angles to toggle
angle_low = "63"
angle_high = "84"

#initializing serial port with baudrate 9600
arduino_port = serial.Serial(port_name, 9600)

def write_servo_high_angle():
    """
    tell servo to go to high angle 
    """
    arduino_port.write((str(angle_high)).encode('utf-8'))


def write_servo_low_angle():
    """
    tell servo to go to high angle 
    """
    arduino_port.write((str(angle_low)).encode('utf-8'))

def close_arduino_port():
    """
    closes arduino port 
    """
    arduino_port.close()
