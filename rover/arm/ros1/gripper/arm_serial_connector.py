#!/usr/bin/env python3

# Original code by: Rudaina Khalil
# Last modified by: Abhay Verma (vabhay.12601@gmail.com)

import serial
import subprocess

# Serial Port object
device_port = 0

class Serial_Port:
    """
    Making a class to deal with serial ports
    """

    def __init__(self, device_name) -> None:
        """
        (String) -> (None)

        Initializing the class by setting device variables. 
        Do an initial attempt at connection right at the start.
        """

        # Setting up some variables
        self.device_port = None
        self.device_name = device_name

    def open_device_port(self, baudrate= 9600):
        """
        (None) -> (None)

        Establishes the connection between the code and the
        serial port.
        """

        # Specifying the command prompt to be run on terminal and running it
        command_prompt  = "/home/rsx/rover_ws/src/rsx-rover/scripts/utils/gen/find_usb.sh"
        process         = subprocess.run(command_prompt, capture_output= True, shell= True)

        # Decoding the output from the command as a list of string
        ports           = process.stdout.decode().split(sep= "\n")
        
        # Checking if our device is present
        for port in ports:
            
            # If it is present, break out of the loop with that name
            if self.device_name in port:
                break

            # Else, keep port as None
            else:
                port = None
        
        # If port is found
        if port:

            # Just get the port_name out of the entire string
            port_name = port.split(sep= " ")[0]

            # Set permissions to read and write the serial port
            command_prompt  = "sudo chmod 666 " + port_name
            process         = subprocess.run(command_prompt, shell= True)

            #initializing serial port with baudrate 9600
            self.device_port = serial.Serial(port_name, baudrate, timeout= 0.1)

        else:

            # Keep global device_port as None
            print("\n" + self.device_name + " cannot be found\n")
        

    def send_bytes(self, data : str):
        """
        (None) -> (None)

        Tell servo to go to high angle (angle_high)
        """

        # if data[-1] != "\n":
        #     data += "\n"
        # print("Sending the following string:", data)
        # Checking if device port is valid
        if self.device_port:

            # Write data value to device port
            # print(data, type(data))
            # self.device_port.reset_output_buffer()
            self.device_port.write(data.encode('utf-8'))

        else:
            print("\nPort not connected")
    
    def read_bytes(self, key='\n') -> str:
        """
        
        """

        if self.device_port:

            # Read data value coming to the device port
            data = self.device_port.read_until(expected=key)
            if data != "":
                return str(data, encoding='utf-8')
            else:
                return None

    def set_permissions(self):
        """
        (None) -> (None)

        A solution recommended online when facing permissions issues with pyserial
        """

        # Set permissions if needed
        cmd_str = "sudo usermod -aG dialout rsx"
        subprocess.run(cmd_str, shell=True)

    def close_device_port(self):
        """
        (None) -> (None)

        Closes device port 
        """

        # Checking if device port is valid
        if self.device_port:

            # Close the device port and set device_port global back to 0
            self.device_port.close()
            self.device_port = None
        
        else:
            print("\nPort not connected\n")


if __name__ == "__main__":
    while True:
        gripper_connection = Serial_Port(device_name= "STMicroelectronics_STM32_STLink_066DFF353542543351022252")
        gripper_connection.open_device_port(baudrate= 115200)
        user_input = input("Position for the motor ")
        gripper_connection.send_bytes(user_input + '\n')
        # connection = serial.Serial(port= "/dev/ttyACM1", baudrate= 115200)
        # connection.open()
        # connection.write(str(user_input + '\n', encoding= 'utf-8'))
        # connection.close()