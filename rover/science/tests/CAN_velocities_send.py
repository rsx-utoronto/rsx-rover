from CAN_utilities import *
# import can

# Instantiate CAN bus
BUS = initialize_bus()

while True: 
    velocity = float(input("Provide an input velocity value (float): "))
    spark_data = float_to_sparkdata(velocity)
    print(spark_data)

    # Send data packet
            
    # Motor number corresponds with device ID of the SparkMAX
    motor_num = 0xC

    id = generate_can_id(dev_id= motor_num, api= CMD_API_SPD_SET) # API WILL BE CHANGED WHEN USING THE POWER (DC) SETTING
    send_can_message(bus= BUS, can_id= id, data= spark_data)
    
