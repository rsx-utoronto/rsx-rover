/* Arduino Code for 180 degree servos
*
* Commented parts of the code can be used for
* repetitive actions
*
* by: Abhay Verma
*/

#include <Servo.h>

#define SERVO_PIN 3                 //TODO: Make sure the number is a 
                                    //      PWM pin that is connected to the servo

Servo test_servo;                   // Servo variable

const int initial_pos = 0           // TODO: initial position for calibration 
int rel_pos           = 0;          // variable to store the relative servo position from initial
// NOTE: 0 <= initial_pos + rel_pos <= 180, always!!

const int step        = 30          // TODO: step for constant repetitive action


void setup() {

  // Serial connection: Comment this part if no serial connection is needed
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Serial connection established!");

  // Attach servo and move to initial position
  test_servo.attach(SERVO_PIN);
  test_servo.write(initial_pos);
  Serial.print("Starting from servo pos: ");
  Serial.println(initial_pos);

  delay(10);
}

void loop() {

  // Loop for moving to a specific position 
  // (comment this part for repetitive actions)
  rel_pos = read_int();
  int goal = initial_pos + rel_pos;
  
  if (goal > 180 || goal < 0) {
    Serial.println("ERROR: Servo position out of bounds!");
  }

  else {
    Serial.print("Moving to relative postion: ");
    Serial.println(rel_pos);
    move_servo(goal);
  }
  
  // Loop for repetitive actions
  // (comment this part for regular control)
  move_servo(initial_pos)
  delay(2000); // 2s delays after each movement
  myservo.write(initial_pos + step);
  delay(2000);
  myservo.write(initial_pos - step);
  delay(2000);
  myservo.write(initial_pos - step);
  delay(2000);
}

/* move_servo()
* (int) -> (None)
*
* Moves the 180 degree servo to the specified angle
*
* parameters:
*   - new_pos (int): The new position value for the servo motor
* 
* return:
*   - None
*/
void move_servo(int new_pos) {

  Serial.print("Moving to servo position: ");
  Serial.println(new_pos);
  test_servo.write(new_pos);
  delay(100); // Change this delay if needed
}


/* read_int()
* (None) -> (int)
*
* This function reads integer values sent through Serial 
* and returns the received integer value. USE IT ONLY WHEN 
* INTEGER VALUES ARE EXPECTED.
* 
* parameters:
*   - None
* 
* return:
*   - int: The value read from Serial
*/
int read_int() {

  int value = rel_pos;

  if (Serial.available()) {
    value = Serial.parseInt()
  }

  Serial.flush()
  return value
}