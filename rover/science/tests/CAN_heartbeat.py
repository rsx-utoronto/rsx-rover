import can
from CAN_utilities import generate_can_id, initialize_bus, CMD_API_NONRIO_HB

# Instantiate CAN bus
BUS = initialize_bus()

hb = can.Message(
		arbitration_id= generate_can_id(
			dev_id= 0x0, 
			api= CMD_API_NONRIO_HB), 
		data= bytes([0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF]), 
		is_extended_id= True,
		is_remote_frame = False, 
		is_error_frame = False
	)

# global BUS

task = BUS.send_periodic(hb, 2, store_task= False)
print("Heartbeat initiated")

while True:
    pass
