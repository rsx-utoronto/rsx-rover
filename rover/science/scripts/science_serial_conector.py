#!/usr/bin/env python3

# =====================================================
#           Science Serial Class
#
#   This file is made to be used by any Science 
#   code to talk to a microcontroller via serial
#   connection using Python. The purpose is to 
#   use pyserial library and define methods that   
#   that Science team (or other teams in RSX) may
#   regularly need.
# ======================================================
# Original code by: Rudaina Khalil
# Last modified by: Abhay Verma
# ======================================================

import serial
import subprocess

# # Serial Port object
# device_port = 0

# Global variable to check if the code is running on Linux environment or not
IS_LINUX = True

class Serial_Port:
    """
    Making a class to deal with serial ports
    """

    def __init__(self, device_name : str) -> None:
        """
        (str) -> (None)

        Initializing the class by setting device variables. 
        Do an initial attempt at connection right at the start.

        parameters:
        - device_name (str): Name that you wish to identify the connected device with
                            if IS_LINUX == True, then device_name should be the name
                            that "find_usb.sh" script gives

        returns:
        - None
        """

        # Setting up some variables
        self.device_port = None
        self.device_name = device_name

    def open_device_port(self, port_name : str = None, baudrate : int = 9600):
        """
        (None) -> (None)

        Establishes the connection between the code and the
        serial port.
        """

        if IS_LINUX:
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

                #initializing serial port with baudrate
                self.device_port = serial.Serial(port_name, baudrate)

            else:

                # Keep global device_port as None
                print("\n" + self.device_name + " cannot be found\n")

            return
        
        else:
            if (not port_name):
                print("ERROR: Port name not provided")
                return

            #initializing serial port with baudrate
            self.device_port = serial.Serial(port_name, baudrate)
            return
        
        

    def send_bytes(self, data : str):
        """
        (str) -> (None)

        Send the string data as bytes over serial with
        utf-8 encoding

        parameters:
        - data (str): The string data to be sent over serial

        returns:
        - None
        """

        # Checking if device port is valid
        if self.device_port:

            # Write data value to device port
            self.device_port.write(data.encode('utf-8'))

        else:
            print("ERROR: Port not connected")
    
    def read_bytes(self, end : str = "\n"):
        """
        (str) -> (str)

        Reads the incoming data over serial as string and returns it.
        Note: Serial keeps reading until the expected line ending characters
        are detected (specified by "end"). Change the expected end if needed.
        parameters:
        - end (str) (optional): Expected line end character(s). The ending 
                                bytes expected are the bytes resulting 
                                from utf-8 encoding of this parameter

        returns:
        - str: Data received from the serial, decoded with "utf-8"
        """

        if self.device_port:

            # Read data value coming to the device port
            data = self.device_port.read_until(expected= bytes(end))
            return str(data, encoding= 'utf-8')

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