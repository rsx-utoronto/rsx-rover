# ENTIRE FILE MAYBE IN MANUAL NODE or in CONTROLLER NODE
import subprocess

#arduino serial port name on linux
port_name = '/dev/ttyACM4'

#angles to toggle
angle_low = "63"
angle_high = "84"

def chmod_arduino_port():
    """
    (None) -> (None)
    Set serial port permissions to read and write
    """
    cmd_str = "sudo chmod u+rw " + port_name 
    subprocess.run(cmd_str, shell=True)

def set_permissions():
    """
    (None) -> (None)
    A solution recommended online when facing permissions issues with serial port (and pyserial)
    """
    cmd_str = "sudo usermod -aG dialout rsx"
    subprocess.run(cmd_str, shell=True)

def initialize_arduino_port():
    """
    (None) -> (None)
    Initialize serial port
    """
    cmd_str = "stty 9600 -F" + port_name
    subprocess.run(cmd_str, shell=True)

def shell_write_servo_low_angle():
    """
    (None) -> (None)
    Tell servo to go to low angle (angle_low) from shell
    """
    cmd_str = "echo "+ angle_low +" > " + port_name
    subprocess.run(cmd_str, shell=True)

def shell_write_servo_high_angle(port_name):
    """
    (None) -> (None)
    Tell servo to go to high angle (angle_high) from shell
    """
    cmd_str = "echo "+ angle_high +" > " + port_name
    subprocess.run(cmd_str, shell=True)
