from arm_serial_connector import *

# Custom Test Names
arduino_name = "1a86_USB2.0-Ser_"
gripper_name = "1a86_USB2.0-Serial"

# arduino_connection = Serial_Port(device_name= arduino_name)
gripper_connection = Serial_Port(device_name= gripper_name)

while 1:
    gripper_connection.open_device_port(baudrate= 9600)
    
    # user_in = input("new pos: ")
    while 1:
        user_in = '100000'
        user_in += 'x'
        print(user_in, type(user_in))
        gripper_connection.send_bytes(user_in)
    
    # gripper_connection.read_bytes()

    gripper_connection.close_device_port()