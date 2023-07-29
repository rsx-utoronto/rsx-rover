# ENTIRE FILE MAYBE IN MANUAL NODE or in CONTROLLER NODE
import serial
import subprocess

#arduino serial port name on linux
arduino_port = 0

#angles to toggle
angle_low = "63 "
angle_high = "84 "

def open_arduino_port():
    """
    (None) -> (String)
    Get port from shell script code
    """

    # Getting the global arduino port name
    global arduino_port

    # Custom Arduino name to be detected
    arduino_name = "1a86_USB2.0-Serial"

    # Specifying the command prompt to be run on terminal and running it
    command_prompt  = ["bash", "/home/rsx/rover_ws/src/rsx-rover/scripts/utils/gen/find_usb.sh"]
    process         = subprocess.run(command_prompt, capture_output= True)

    # Decoding the output from the command as a list of string
    ports           = process.stdout.decode().split(sep= "\n")

    # Checking if our Arduino is present
    for port in ports:

        # If it is present, break out of the loop with that name
        if arduino_name in port:
            break

        # Else, keep port as None
        else:
            port = None
    
    # If port is found
    if port:

        # Just get the port_name out of the entire string
        port_name = port.split(sep= " ")[0]

        #initializing serial port with baudrate 9600
        arduino_port = serial.Serial(port_name, 9600)

    else:

        # Keep global arduino_port as 0
        arduino_port = 0
        print("\nArm Arduino is not connected\n")

def write_servo_high_angle():
    """
    (None) -> (None)
    Tell servo to go to high angle (angle_high)
    """

    # Write servo value to Arduino port
    arduino_port.write((str(angle_high)).encode('utf-8'))


def write_servo_low_angle():
    """
    (None) -> (None)
    Tell servo to go to low angle (angle_low)
    """

    # Write servo value to Arduino port
    arduino_port.write((str(angle_low)).encode('utf-8'))

def set_permissions():
    """
    (None) -> (None)
    A solution recommended online when facing permissions issues with pyserial
    """

    # Set permissions if needed
    cmd_str = "sudo usermod -aG dialout rsx"
    subprocess.run(cmd_str, shell=True)

def close_arduino_port():
    """
    (None) -> (None)
    Closes arduino port 
    """

    # Close the Arduino port and set arduino_port global back to 0
    arduino_port.close()
    arduino_port = 0


#initializing serial port with baudrate 9600
#arduino_port = serial.Serial(port_name, 9600)

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