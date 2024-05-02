#include <Arduino.h>
#include <Wire.h>

#define slave_address 2
int n = 2; //number of motors and in this case also number of interupts.
const int encoder_a_pins[2] = {9,5}; //Orange      // encoder 0 is base/link1, encoder 1 is link1/link2    A is green B is yellow.
const int encoder_b_pins[2] = {10,6}; //Brown

volatile int bLastStates[2] = {0};
volatile int aLastStates[2] = {0};
volatile int aStates[2] = {0};
volatile int bStates[2] = {0};
long int count_values_last[2] = {0,0}; 
long int count_values[2] = {0,0}; 
boolean setup_lim = false;


void receiveEvent(int numBytes);
void requestEvent();
void isr();

void setup() {
    Wire.begin(slave_address);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
 
    // Setting all pinModes and Sensor states
    for (int i = 0; i < n; i++) {
        pinMode(encoder_a_pins[i], INPUT);
        pinMode(encoder_b_pins[i], INPUT);
        //pinMode(limit_pins[i], INPUT_PULLUP);     //no limit switches
        attachInterrupt(digitalPinToInterrupt(encoder_a_pins[i]), isr,CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoder_b_pins[i]), isr,CHANGE);
    }
}


void loop(){
  if(setup_lim){
    count_values_last[0] =-1000;
    count_values_last[1] =-1000;
    count_values[0]= 20000;
    delay(1000);
    while(count_values_last[0]!=count_values[0]){
      count_values_last[0]=count_values[0];
      delay(100);
    }
    count_values[0] = 0;
    count_values[1] = 20000;
    Serial.println(count_values[1]);
    delay(1000);
    while(count_values_last[1]!=count_values[1]){
      count_values_last[1]=count_values[1];
      delay(100);
    }
    Serial.println(count_values[1]);
    count_values[1] = 0;
    setup_lim = false;
  }
}



void isr(){
    // Update each sensor's state
    //Serial.println("Spinning!");
    
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
        out_bytes[6 * i] = (i+1) & 0xFF;                             // First index contains sensor number
        out_bytes[6 * i + 1] = (1) & 0xFF;                          // Second index contains limit switch value (1 or 0)  // no limit switches 
        out_bytes[6 * i + 2] = (count_values[i] >> 24) & 0xFF;     // Third index contains most significant 8 bits of count values  
        out_bytes[6 * i + 3] = (count_values[i] >> 16) & 0xFF;     // Fourth contains next 8 bits and so on.....
        out_bytes[6 * i + 4] = (count_values[i] >> 8) & 0xFF;
        out_bytes[6 * i + 5] = count_values[i] & 0xFF;
    }
    Wire.write(out_bytes, 6*n);
    digitalWrite(LED_BUILTIN, 0);
}
