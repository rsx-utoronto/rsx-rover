import arm_servo as servo

while 1: 
    trigger = int(input("1 or 0\n"))
    if trigger:
        servo.write_servo_high_angle()
    else:
        servo.write_servo_low_angle()