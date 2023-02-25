import arm_can_control as arm_can
from GetManualJoystickFinal import *
from MapManualJoystick import *
import struct

def pos_to_sparkdata(f : float):
    """
    float -> list()

    Takes in a float position value and returns the data packet in the form that
    SparkMAX requires

    @parameters:

    f (float): Float value that needs to be converted
    """
    input_hex =  hex(struct.unpack('<I', struct.pack('<f', f))[0])
    if len(input_hex) != 10:
        input_hex = input_hex[:2] + input_hex[2:] + (10-len(input_hex))*'0'
    
    return [eval('0x'+input_hex[-2:]), eval('0x'+input_hex[-4:-2]),
            eval('0x'+input_hex[-6:-4]), eval('0x'+input_hex[-8:-6]),
            0x00, 0x00, 0x00, 0x00]

def generate_data_packet (data_list : list):
    """
    list(float) -> list(list(int))

    Takes in the goal position angles for each motor and converts them to bytearrays specific for
    each motor
    """
    
    # A variable to keep count of joint number
    joint_num = 0
    
    # Sparkmax Data list
    spark_data = []

    for angle in data_list:
        joint_num += 1
        
        # Gear reduction
        reduction = 100

        if joint_num == 1:
            angle = angle/360
            spark_data.append(pos_to_sparkdata(angle))
        
        elif joint_num == 2:
            angle = angle/360
            spark_data.append(pos_to_sparkdata(angle))

        elif joint_num == 3:
            angle = angle/360
            spark_data.append(pos_to_sparkdata(angle))
        
        elif joint_num == 4:
            angle = angle/360
            spark_data.append(pos_to_sparkdata(angle))
        
        elif joint_num == 5:
            angle = angle/360
            spark_data.append(pos_to_sparkdata(angle))

        elif joint_num == 6:
            angle = angle/360
            print("motor 6 pos", angle)
            spark_data.append(pos_to_sparkdata(angle))

        elif joint_num == 7:
            angle = angle/360
            spark_data.append(pos_to_sparkdata(angle))

        else:
            print("Error: Reaching infinite loop/Out of Index")
            break
    print(spark_data)
    return spark_data

if __name__=="__main__":
    
    # Instantiating the CAN bus
    arm_can.initialize_bus()

    # Creating the message packet for Heartbeat
    hb = arm_can.can.Message(
        arbitration_id= arm_can.generate_can_id(
            dev_id= 0x0, 
            api= arm_can.CMD_API_NONRIO_HB), 
        data= bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]), 
        is_extended_id= True,
        is_remote_frame = False, 
        is_error_frame = False
    )

    # Broadcasting the heartbeat
    task = arm_can.BUS.send_periodic(hb, 0.01)
    print("Heartbeat initiated")

    # Starting the infinite loop
    while 1:

        # Getting angles from the remote controller
        input_angles = MapJoystick(GetManualJoystickFinal.GetManualJoystick(), current_pos, speed_limit, t-time.time())
        
        print(input_angles)
        
        # Converting received angles to SparkMAX data packets
        spark_input = generate_data_packet(input_angles)
        print("Motor 6 spark input", spark_input[5])

        # Sending data packets one by one
        for i in range(1, len(spark_input)):
            
            # Motor number corresponds with device ID of the SparkMAX
            motor_num = 11 + i

            if motor_num > 10 and motor_num < 18:
                id = arm_can.generate_can_id(dev_id= motor_num, api= arm_can.CMD_API_POS_SET)
                arm_can.send_can_message(can_id= id, data= spark_input[i])
            
            else:
                break

        t = time.time()
        time.sleep(.1)
