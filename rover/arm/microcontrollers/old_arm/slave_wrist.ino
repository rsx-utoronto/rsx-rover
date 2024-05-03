#include <Arduino.h>
#include <Wire.h>

#define slave_address 3                             // there are 3 slaves: base slave(1), link one sleave(2), link three slave(3).
int n = 4;                                          //number of motors and in this case also number of interupts.
int failsafe = 0;
int numOfSensorsBelow = 3;                          //number of sensors controlled by slaves one and two.
const int encoder_a_pins[4] = {11,10,8,14}; //G     // motor 0 is wrist, motors 1,2 are bevel, motor 3 is gripper, is dependent on hardware but can be changesd.
const int encoder_b_pins[4] = {12,9,7,6}; //Y
const int limit_pins[4] = {5,4,3,2};                // switch 0, 1 is wrist, switch 2,3 is bevel, gripper has no switch atm.
const int error = 30;
volatile int bLastStates[4] = {0,0,0,0};            // last states of a pins.
volatile int aLastStates[4] = {0,0,0,0};            // last states of b pins.
volatile int aStates[4] = {0,0,0,0};                // current (as of an interupt) a states.
volatile int bStates[4] = {0,0,0,0};                // current (as of an interupt) b states.
volatile int limitLastStates[4] = {1,1,1,1};
long int count_values[4] = {0,0,0,0};               // a negative number unfolds the arm, a possitive folds it into its most compact state.    
                                                    // it is desired for the arm to unfold and ensure that it backs away from every limit switch.
boolean wrist_setup = false;
boolean bevel_setup = false;
boolean gripper_setup = false; 
boolean setup_lim = false;


void receiveEvent(int numBytes);
void requestEvent();
void isr();
void lim();
void lim_setup();
void lim_bevel();
void lim_wrist();

void setup() {
    Wire.begin(slave_address);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
 
    // Setting all pinModes and Sensor states
    for (int i = 0; i < n; i++) {
        count_values[i]=0;
        pinMode(encoder_a_pins[i], INPUT);
        pinMode(encoder_b_pins[i], INPUT);
        pinMode(limit_pins[i], INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(encoder_a_pins[i]), isr,CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoder_b_pins[i]), isr,CHANGE);
    }
    for (int i = 0; i < 2; i++) {
      attachInterrupt(digitalPinToInterrupt(limit_pins[i]), lim_wrist,CHANGE);
      attachInterrupt(digitalPinToInterrupt(limit_pins[i+2]), lim_bevel,CHANGE);
    }
}

void loop(){
  if (Serial.available()) {
        count_values[0] = Serial.parseInt();
        Serial.println(count_values[0]); 
  }
  if(setup_lim){
    lim_setup();
  }
  lim();
  //Serial.println(count_values[0]);
}

void lim_setup(){
  count_values[0] = -500;
  while(count_values[0]< -error && failsafe < 100){
    failsafe++;
    //Serial.println(failsafe);
    delay(10);
  }
  failsafe = 0;
  count_values[0] = 20000;
  Serial.println(failsafe);
  while(count_values[0] > error){
    //wait to hit limit switch
    Serial.println(" ");
  }
  Serial.println(failsafe);
  count_values[1] = -500;
  count_values[2] = -500;
  Serial.println(failsafe);
  while(count_values[1]< -error && count_values[2]< -error && failsafe < 100){
    failsafe++;
    delay(10);
  }

  failsafe = 0;
  count_values[1] = 20000;
  count_values[2] = 20000;
  while(count_values[1] > error){
    //wait to hit limit switch
    Serial.println();
  }
  setup_lim = false;
}

void isr(){
    // Update each sensor's state
    for (int i = 0; i < n; i++) {
        aStates[i] = digitalRead(encoder_a_pins[i]);  // Reads the "current" state
        bStates[i] = digitalRead(encoder_b_pins[i]);
        // compare to previous state, increment or decrement
        if (aStates[i] != aLastStates[i]) {
            if (aStates[i]) {
                bStates[i] ? count_values[i]-- : count_values[i]++;
            } else {
                bStates[i] ? count_values[i]++ : count_values[i]--;
            }
        }

        if(bLastStates[i] != bStates[i]) {
            if(bStates[i]) {
                aStates[i] ? count_values[i]++ : count_values[i]--;
            } else {
                aStates[i] ? count_values[i]-- : count_values[i]++;
            }
        }
        bLastStates[i] = bStates[i];
        aLastStates[i] = aStates[i];
    }
}

void lim_wrist(){
  if(limitLastStates[0]!=digitalRead(limit_pins[0])){
    count_values[0] = 0;
    wrist_setup = true;
  }
  limitLastStates[0] = digitalRead(limit_pins[0]);
  limitLastStates[1] = digitalRead(limit_pins[1]);
}

void lim_bevel(){
  if(limitLastStates[2]!=digitalRead(limit_pins[2])){
    count_values[1] = 0;
    count_values[2] = 0;
    bevel_setup = true;
  }
  limitLastStates[2] = digitalRead(limit_pins[2]);
  limitLastStates[3] = digitalRead(limit_pins[3]);
}

void lim(){
  for(int i=0; i<n; i++){
    if(limitLastStates[i]==0){
      delay(100);
      limitLastStates[i]=1;
    }
  }
}

void receiveEvent(int numBytes) {
    setup_lim=true;
    Serial.println("recieved");
    while(Wire.available()){
      Wire.read();
    }
}

void requestEvent() { // Report back the value of the sensor requested
    digitalWrite(LED_BUILTIN, 1);
    byte out_bytes[6*n];
    for (int i = 0; i < n; i++) {
        out_bytes[6 * i] = (i + numOfSensorsBelow) & 0xFF;         // First index contains sensor number
        out_bytes[6 * i + 1] = (limitLastStates[i]) & 0xFF;        // Second index contains limit switch value (1 or 0)    
        out_bytes[6 * i + 2] = (count_values[i] >> 24) & 0xFF;     // Third index contains most significant 8 bits of count values  
        out_bytes[6 * i + 3] = (count_values[i] >> 16) & 0xFF;     // Fourth contains next 8 bits and so on.....
        out_bytes[6 * i + 4] = (count_values[i] >> 8) & 0xFF;
        out_bytes[6 * i + 5] = count_values[i] & 0xFF;
    }
    Wire.write(out_bytes, 6*n);
    digitalWrite(LED_BUILTIN, 0);
}
