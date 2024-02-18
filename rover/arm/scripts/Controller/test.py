from arm_serial_connector import *

# Custom Test Names
arduino_name = "1a86_USB2.0-Ser_"
gripper_name = "STMicroelectronics_STM32_STLink_066DFF485671664867172828"

arduino_connection = Serial_Port(device_name= arduino_name)
gripper_connection = Serial_Port(device_name= gripper_name, baudrate= 115200)

while 1:
    user_in = input("1 for direction, 2 for on/off: ")

    gripper_connection.send_bytes(user_in)