import arm_servo as servo
from MapManualJoystick import *

triggered = 0

while 1: 
    trigger = MapJoystick(GetManualJoystickFinal.GetManualJoystick(), current_pos, speed_limit, t-time.time())[-1]
    print(trigger)
    if trigger and triggered == 0:
        print("Triggering")
        servo.write_servo_high_angle()
        triggered = 1
    elif trigger == 0 and triggered == 1:
        print("Triggering Again")
        servo.write_servo_low_angle()
        triggered = 0