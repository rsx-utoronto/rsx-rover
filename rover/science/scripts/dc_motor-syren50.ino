
/*      Syren50 Motor Controller Code
*
* This code is to be used for testing the DC motor connected to 
* Syren50 motor controller in Analog Input mode (Mode 1) and 
* Option 3 (Analog One-direction with forward/reverse select on S2).
*
* For more details on the motor controller, check out:
* https://www.dimensionengineering.com/datasheets/SyRen50.pdf
*
* by Axel Pena Hernandez
* edited by Abhay Verma
*/
#include <ctype.h>


#define MOTOR_DIR_PIN  8      // TODO: Make sure the pins are connected correctly
#define MOTOR_VEL_PIN  9       

char input = '\0' ;


void setup() {

  // Serial connection: Comment this part if no serial connection is needed
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Serial connection established!");
  Serial.println("Type 'd' or 'D' to change directions");
  Serial.println("Type 's' or 'S' to change speed");

  // Setting Pin modes and initial configurations
  pinMode(MOTOR_VEL_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  
  digitalWrite(MOTOR_DIR_PIN, HIGH);  // Whether to start it as a 
                                      // HIGH or LOW depends on how the motor is powered
  digitalWrite(MOTOR_VEL_PIN, LOW);
}


void loop() {
  if (Serial.available()) {
    input = Serial.read();
    Serial.print("INPUT: ");
    Serial.println(toupper(input));
    input = toupper(input);
  }

  else {
    input = '\0';
  }
  
  if (input != 'D' && input != 'S') {
    Serial.println("ERROR: Invalid Input!");
  }

  // set the velocity of pin 9:
  switch(input) {
    case 'D':
      digitalWrite(MOTOR_DIR_PIN, !digitalRead(MOTOR_DIR_PIN));
      Serial.println("Direction flipped");
      break;

    case 'S':
      digitalWrite(MOTOR_VEL_PIN, !digitalRead(MOTOR_VEL_PIN));
      Serial.println("Speed Flipped");
      break;
  }

  input = '\0';
  delay(10);
}
