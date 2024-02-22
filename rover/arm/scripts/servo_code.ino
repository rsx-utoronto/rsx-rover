#include <Servo.h>
#define LED_PIN 13

Servo myServo;
int angle     = 63;
//int triggered = 0;

void setup() {
  Serial.begin(9600);
  myServo.attach(5);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  if (Serial.available() >= 2) {
    angle = Serial.parseInt();
    Serial.print("Servo angle set to: ");
    Serial.println(angle);
    if (angle == 84) {
      digitalWrite(LED_PIN, HIGH);
    }
    else {
      digitalWrite(LED_PIN, LOW);
    }
    //delay(10); // Delay to give the servo time to move
    Serial.flush();
  }
  myServo.write(angle);
  delay(1);
}
