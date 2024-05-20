#include <Arduino.h>
#include <Wire.h>
//#include <main.h>

#define slave_address 1                         // base slave (1)
int n = 1;                                      //number of motors and in this case also number of interupts.
int failsafe = 0;
const int encoder_a_pins[1] = {3}; //Blue       // encoder 0 is base motor
const int encoder_b_pins[1] = {4}; //Blue
const int limit_pins[1] = {9};     //Green      // Bumper limit switch
const int error = 30;

volatile int bLastStatkes[1] = {0};              //last states of a pins
volatile int aLastStates[1] = {0};              //last states of b pins
volatile int aStates[1] = {0};
volatile int bStates[1] = {0};
volatile int limitLastStates[1] = {1};              
long int count_values[1] = {0};                 //insert # of motors

boolean base_setup = false;
boolean setup_lim = false;

void receiveEvent(int numBytes);
void requestEvent();
void isr();
void lim();
void lim_setup();
void lim_base();

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
    for (int i =0; i < 1; i++) {
      attachInterrupt(digitalPinToInterrupt(limit_pins[i]), lim_base, FALLING);
    }
    
    
}

void loop(){
  if (Serial.available()){
     count_values[0] = Serial.parseInt();
     Serial.println(count_values[0]);
  }
  if (setup_lim){
    lim_setup();
  }
  lim();
}

void lim_setup() {
  //setting motor's position 0 with base bumper limit switch

  //move in case currently on limit switch
  count_values[0] = -500;
  while(count_values[0] < -error && failsafe < 100){
    failsafe++;
    //Serial.println(failsafe);
    delay(10);
  }
  failsafe = 0;

  //move to hit limit switch
  count_values[0] = 40000;
  Serial.println(failsafe);
  while(count_values[0] > error) {
    //wait to hit limit switch
    Serial.println(" ");
  }
  setup_lim = false;
}


void isr(){
    // Update each sensor's state
    
    for (int i = 0; i < n; i++) {
        aStates[i] = digitalRead(encoder_a_pins[i]);  // Reads the "current" state
        bStates[i] = digitalRead(encoder_b_pins[i]);
        Serial.println(count_values[i]);
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

void lim_base(){
  
  if(limitLastStates[0] != digitalRead(limit_pins[0])){
    count_values[0] = 0;
    base_setup = true;
  }
  limitLastStates[0] = digitalRead(limit_pins[0]);
  
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
    setup_lim = true;
    Serial.println("received");
    while(Wire.available()){
      Wire.read();
    }
}

void requestEvent() { // Report back the value of the sensor requested
    digitalWrite(LED_BUILTIN, 1);
    byte out_bytes[6*n];
    for (int i = 0; i < n; i++) {
        out_bytes[6 * i] = (i) & 0xFF;                             // First index contains sensor number
        out_bytes[6 * i + 1] = (limitLastStates[i]) & 0xFF;        // Second index contains limit switch value (1 or 0)    
        out_bytes[6 * i + 2] = (count_values[i] >> 24) & 0xFF;     // Third index contains most significant 8 bits of count values  
        out_bytes[6 * i + 3] = (count_values[i] >> 16) & 0xFF;     // Fourth contains next 8 bits and so on.....
        out_bytes[6 * i + 4] = (count_values[i] >> 8) & 0xFF;
        out_bytes[6 * i + 5] = count_values[i] & 0xFF;
    }
    Wire.write(out_bytes, 6*n);
    digitalWrite(LED_BUILTIN, 0);
}
