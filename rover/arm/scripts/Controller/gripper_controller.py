# Original Code by: Rudaina Khalil
# Latest Modification by: Abhay Verma (vabhay.1601@gmail.com)

from arm_serial_connector import *

gripper_name = "STMicroelectronics_STM32_STLink_066DFF485671664867172828"

gripper_connection = Serial_Port(device_name= gripper_name)




#initializing serial port with baudrate 9600
#gripper_port = serial.Serial(port_name, 9600)

# triggered = 0

while 1:
    # Getting angles from the remote controller
        
        open_gripper_port()
        set_permissions()
        write_servo_low_angle()
        print("Set low")
    