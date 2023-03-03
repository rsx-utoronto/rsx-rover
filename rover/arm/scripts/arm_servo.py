import serial
import subprocess

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
    arduino_port.write((str(angle_high)).encode('utf-8'))


def write_servo_low_angle():
    """
    (None) -> (None)
    Tell servo to go to low angle (angle_low)
    """
    arduino_port.write((str(angle_low)).encode('utf-8'))

def set_permissions():
    """
    (None) -> (None)
    A solution recommended online when facing permissions issues with pyserial
    """
    cmd_str = "sudo usermod -aG dialout rsx"
    subprocess.run(cmd_str, shell=True)

def close_arduino_port():
    """
    (None) -> (None)
    Closes arduino port 
    """
    arduino_port.close()
