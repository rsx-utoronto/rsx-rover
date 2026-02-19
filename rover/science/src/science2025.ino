#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <OneWire.h>
#include <string.h>

#define DEBUG

/*
 KEYMAP

 I: Focus button for microscope camera - currently not operating
 O: Focus button for microscope camera - currently not operating

 S: Switch PMT position

 N: Release nihydrin (toggle switch)

 M: Rotate filter wheel

 A: Rotate servo for chem test 1
 B: Rotate servo for chem test 2

 D: Rotate servo for FAD collection 1
 E: Rotate servo for FAD collection 2

 X: Toggle LD main light
 Y: Toggle LD for FAD collection 1
 Z: Toggle LD for FAD collection 2
*/

// List of functions
void recieveCommand();
int move_to_position(int cur_position, int new_position, int stepsPerRev, int num_positions, int stepPin, int dirPin);
void step(int num_steps, int stepPin, int dirPin);
void stopAll();
void my_delay(int delay_time);
void serialFlush();
void fillBuffer();
void getTempHumid();
float getTemp();
void arduinoHWReset();

/* DATA BUFFERS */
const int input_size                    = 32;
char input_text[input_size];
char command[16]                        = {0};
int specification                       = 0;
boolean new_data                        = false;

/* SERVO SPECIFIC VARIABLES */
unsigned long int filter_start_time     = 0;
unsigned long int filter_max_time       = 100000000;
boolean filter_started                  = false;
unsigned long int microscope_start_time = 0;
unsigned long int microscope_max_time   = 100000000;
boolean chem1_started                   = false;
boolean chem2_started                   = false;
boolean fam1_started                    = false;
boolean fam2_started                    = false;
const int pmt_pos1                      = 0;
const int pmt_pos2                      = 0;
int curr_pmt_pos                        = 0;
int dir                                 = 0;

/* SERVO PINS */
const int microscope_pin                = 6;
const int pmt_pin                       = 7;
const int filter_pin                    = 8;
const int chem1_pin                     = 9;
const int chem2_pin                     = 10;
const int fam1_pin                      = 11;
const int fam2_pin                      = 12;

/* HIMIDITY DATA */
const int AirValue = 424;   //you need to replace this value with Value_1
const int WaterValue = 195;  //you need to replace this value with Value_2

/* ANALOG PINS */
const int PMT_sig                       = A0;
const int PMT_Vref                      = A1;
const int pH1_pin                       = A2;
const int pH2_pin                       = A3;
const int hum_pin                       = A15;


/* DIGITAL I/O PINS */
const int NH_pin                        = 30;
const int ld_full_pin                   = 31;
const int ld1_pin                       = 32;
const int ld2_pin                       = 33;
const int reset_pin                     = 34;

/* SERVO OBJECTS*/
Servo servo_microscope;
Servo servo_pmt;
Servo servo_filter;
Servo servo_chem1;
Servo servo_chem2;
Servo servo_fam1;
Servo servo_fam2;

/* TEMP SENSOR */
int DS18B20_Pin = 13;   // temp only sensor pin (skinny wires in URC config)
OneWire ds(DS18B20_Pin);



void setup() {

  /* INITIALIZE SERIAL */
  Serial.begin(9600);

  /* SETTING UP SERVOS */
  servo_microscope.attach(microscope_pin);
  servo_pmt.attach(pmt_pin);
  servo_filter.attach(filter_pin);
  servo_chem1.attach(chem1_pin);
  servo_chem2.attach(chem2_pin);
  servo_fam1.attach(fam1_pin);
  servo_fam2.attach(fam2_pin);

  /* DIGITIAL PIN SETUPS */
  digitalWrite(NH_pin, LOW);
  digitalWrite(ld_full_pin, LOW);
  digitalWrite(ld1_pin, LOW);
  digitalWrite(ld2_pin, LOW);
  digitalWrite(reset_pin, HIGH);
  
  pinMode(NH_pin, OUTPUT);
  pinMode(ld_full_pin, OUTPUT);
  pinMode(ld1_pin, OUTPUT);
  pinMode(ld2_pin, OUTPUT);
  pinMode(reset_pin, OUTPUT);
  
}

void loop() {
  
  read_input();
  
  // if no new data received, try again
  if (new_data) {
    parse_input();
    new_data = false;
  
    if (strlen(command) != 0) {
  
      switch(command[0]) {
        case 'I':
          dir = 1;
          rotate_servo(servo_microscope, specification, dir);
          microscope_start_time = millis();
          break;
      
        case 'O':
          dir = -1;
          rotate_servo(servo_microscope, specification, dir);
          microscope_start_time = millis();
          break;
      
        case 'S':
          toggle_pmt_servo(servo_pmt);
          break;
      
        case 'N':
          digitalWrite(NH_pin, !digitalRead(NH_pin));
    
        case 'M':
          dir = 1;
          specification = 10
          rotate_servo(servo_filter, specification, dir);
          filter_start_time = millis();
          filter_started = true;
          break;
    
        case 'A':
          dir = 1;
          specification = 10;
          if (chem1_started) {
            rotate_servo(servo_chem1, 0, dir);
            chem1_started = false;
          }
          else {
            rotate_servo(servo_chem1, specification, dir);
            chem1_started = true;
          }
          specification = 0;
          break;
      
        case 'B':
          dir = 1;
          specification = 10;
          if (chem2_started) {
            rotate_servo(servo_chem2, 0, dir);
            chem2_started = false;
          }
          else {
            rotate_servo(servo_chem2, specification, dir);
            chem2_started = true;
          }
          specification = 0;
          break;
      
        case 'D':
          dir = 1;
          specification = 10;
          if (fam1_started) {
            rotate_servo(servo_fam1, 0, dir);
            fam1_started = false;
          }
          else {
            rotate_servo(servo_fam1, specification, dir);
            fam1_started = true;
          }
          specification = 0;
          break;
      
        case 'E':
          dir = 1;
          specification = 10;
          if (fam2_started) {
            rotate_servo(servo_fam2, 0, dir);
            fam2_started = false;
          }
          else {
            rotate_servo(servo_fam2, specification, dir);
            fam2_started = true;
          }
          specification = 0;
          break;
      
        case 'X':
          digitalWrite(ld_full_pin, !digitalRead(ld_full_pin));
          break;
    
        case 'Y':
          digitalWrite(ld1_pin, !digitalRead(ld1_pin));
          break;
      
        case 'Z':
          digitalWrite(ld2_pin, !digitalRead(ld2_pin));
          break;
      
        default:
          Serial.println("ERROR: UNKNOWN COMMAND");
          break;
      }
    }
  }

  if (millis() - filter_start_time >= filter_max_time and filter_started {
      //millis() - microscope_start_time >= microscope_max_time){

    specification = 0;
    rotate_servo(servo_filter, specification, dir);
    filter_started = false;
//    Serial.println("ERROR: RESETTING");    
//    digitalWrite(reset_pin, LOW);
  }

  float pH1_data = pH(pH1_pin);
  float pH2_data = pH(pH2_pin);
  float temp_data = getTemp();
  float hum_data = humidity();
  float PMT_data[2] = {0};
  PMT(PMT_data);

  Serial.print("pH1: ");
  Serial.print(pH1_data);
  Serial.print(";pH2: ");
  Serial.print(pH2_data);
  Serial.print(";Hum: ");
  Serial.print(hum_data);
  Serial.print(";Temp: ");
  Serial.print(temp_data);
  Serial.print(";PMT: ");
  Serial.print(PMT_data[0]);
  Serial.print(";Pos: ");

  if (curr_pmt_pos == pmt_pos1) Serial.println(1);
  else if (curr_pmt_pos == pmt_pos2) Serial.println(2);

  Serial.flush();
  delay(100);
}

void read_input() {
  char start_char = '<';
  char end_char = '>';
  int index = 0;
  boolean recv_in_progress = false;
  
  while (Serial.available() > 0 && new_data == false) {
    char input = Serial.read();

    if (recv_in_progress == false && input == start_char) recv_in_progress = true;

    else if (recv_in_progress) {
      if (input != end_char) {
        input_text[index] = input;
        index++;
        if (index >= input_size) index--; 
      }
      else {
        input_text[index] = '\0';
        new_data = true;
      }
    }
  }
}

void parse_input() {
  char* token;
  token = strtok(input_text, ",");
  strcpy(command, token);

  token = strtok(NULL, ",");
  if (token != NULL) specification = atoi(token);

  Serial.print("RECEIVED Ccommand: ");
  Serial.print(command);
  Serial.print(" and Specification: ");
  Serial.println(specification);
  Serial.flush();
  delay(100);
}




//int move_to_position(int cur_position, int new_position, int stepsPerRev, int num_positions, int stepPin, int dirPin){
//  Serial.println("MOVE TO POSITION HAS BEEN CALLED");
//
//  // This function is used by both the bit changer and test tube carousel
//
//  // Bit changer: 
//  // 30 DEGREES BETWEEN BIT CHANGER HOLES -> 12 POSITIONS TOTAL
//  // bits are at positions are 0, 1, 2, 3, 4, 5, 6
//  // hole is at position 9, opposite the rest of the bits
//
//  // Test tube carousel:
//  // 25.714 DEGREES BETWEEN BIT CHANGER HOLES -> 14 POSITIONS TOTAL
//
//  int num_steps = int((float(new_position - cur_position) * stepsPerRev) / num_positions);
//  
//  step(num_steps, stepPin, dirPin);
//  Serial.println();
//  Serial.print("STEPS ADVANCED: ");
//  Serial.println(num_steps);
//  return new_position;
//}
//
//void step(int num_steps, int stepPin, int dirPin){
//  if (num_steps > 0){
//    digitalWrite(dirPin,HIGH);
//    Serial.println("Forwards");
//  }
//  else{
//    digitalWrite(dirPin,LOW);
//    Serial.println("Backwards");
//  }
//  for(int x = 0; x < abs(num_steps); x++) {
//    digitalWrite(stepPin,HIGH); 
//    delay(1); 
//    digitalWrite(stepPin,LOW); 
//    delay(1);
//    if (x % 100 ==0){
//      fillBuffer(); // despite the small time step we still need to check the buffer for a stop signal periodically
//    }
//  }
//}
//
//void stopAll(){
//  drill_falcon.writeMicroseconds(1500); //Drill
//  drill_LA.writeMicroseconds(1500); //Linear actuator
//}

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}


//void getTempHumid(){
//  for (int j = 0; j <10; j++){
//    float test_temp = getTemp();
//
//    Serial.print(F("Sensor: "));
//    Serial.print(test_temp);
//
//    float h = dht.readHumidity();
//    // Read temperature as Celsius (the default)
//    float t = dht.readTemperature();
//    // Read temperature as Fahrenheit (isFahrenheit = true)
//    float f = dht.readTemperature(true);
//
//    // Check if any reads failed and exit early (to try again).
//    if (isnan(h) || isnan(t) || isnan(f)) {
//      Serial.println(F("Failed to read from DHT sensor!"));
//    }
//
//    // Compute heat index in Fahrenheit (the default)
//    float hif = dht.computeHeatIndex(f, h);
//    // Compute heat index in Celsius (isFahreheit = false)
//    float hic = dht.computeHeatIndex(t, h, false);
//
//    Serial.print(F(" Humidity: "));
//    Serial.print(h);
//    Serial.print(F("%  Temperature: "));
//    Serial.print(t);
//    Serial.print(F("°C "));
//    Serial.print(f);
//    Serial.print(F("°F  Heat index: "));
//    Serial.print(hic);
//    Serial.print(F("°C "));
//    Serial.print(hif);
//    Serial.println(F("°F"));
//
//    my_delay(1000);
//  }
//}

void rotate_servo(Servo &servo, int spec, int dir) {
  int vel = 90 + dir * spec * 9;
  servo.write(vel);
  delay(100);                       
}

void toggle_pmt_servo(Servo &servo) {
  
  if (curr_pmt_pos == pmt_pos2) {
    servo.write(pmt_pos1);
    curr_pmt_pos = pmt_pos1;
  }

  if (curr_pmt_pos == pmt_pos1) {
    servo.write(pmt_pos2);
    curr_pmt_pos = pmt_pos2;
  }
}

// Function for Getting Temperature
float getTemp(void) {
    byte data[16];
    byte addr[8];


    if (!ds.search(addr)){
        ds.reset_search();
        return -1000;
    }

    if(OneWire::crc8(addr, 7) != addr[7]){
        Serial.println("CRC is not valid");
        return -2000;
    }

    if(addr[0] != 0x10 && addr[0] != 0x28){
        Serial.println("Device is not recognized");
        return -3000;
    }

    ds.reset();
    ds.select(addr);
    ds.write(0x44,1);

    byte present = ds.reset();
    ds.select(addr); 
    ds.write(0xBE); // read scratchpad

    for (int i = 0; i < 9; i++){
        data[i] = ds.read();
    }

    ds.reset_search();

    byte MSB = data[1];
    byte LSB = data[0];

    float tempRead = ((MSB<<8)|LSB);
    float TemperatureSum = tempRead / 16;

    return TemperatureSum;
}


float pH (int pH_pin) {
  float measure = analogRead(pH_pin);
  //Serial.print("Measure: ");
  //Serial.print(measure);

  double voltage = 5 / 1024.0 * measure; //classic digital to voltage conversion
  //Serial.print("\tVoltage: ");
  //Serial.print(voltage, 3);

  // PH_step = (measure@PH7 - measure@PH4) / (PH7 - PH4)
  // PH_probe = PH7 - ((measure - measure@PH7) / PH_step)
  //float Po = ((measure - 959) / -21) +7;
  //float Po = ((measure - 1022) / -63)*3 +4;
  float Po = ((measure - 1022) / -27)*3 +4; // corrected calibration
  return Po; 
}



float humidity() {
  float soilMoistureValue = analogRead(hum_pin);  //put Sensor insert into soil
  float rel_hum = 0;
  
//  Serial.print("Soil Moisture (Absolute): ");
//  Serial.println(soilMoistureValue);
  
  float soilmoisturepercent = map(soilMoistureValue, AirValue, WaterValue, 0, 100);
  if(soilmoisturepercent >= 100)
  {
//    Serial.println("100 %");
    rel_hum = 100;
  }
  else if(soilmoisturepercent <=0)
  {
//    Serial.println("0 %");
    rel_hum = 0;
  }
  else if(soilmoisturepercent > 0 && soilmoisturepercent < 100)
  {
//    Serial.print("Soil Moisture (Relative Humidity): ");
//    Serial.print(soilmoisturepercent);
//    Serial.println("%\n");
    rel_hum = soilmoisturepercent;
  }

  return rel_hum;
}

void PMT(float* PMT_data) {
  // read the input on analog pin 0:
  int sensorValue1 = analogRead(PMT_sig);
  int sensorValue2 = analogRead(PMT_Vref);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  PMT_data[0] = sensorValue1 * (5.0 / 1023.0);
  PMT_data[1] = sensorValue2 * (5.0 / 1023.0);
  // print out the value you read:
  
//  Serial.print("A0: ");
//  Serial.print(voltage1);
//  Serial.print(" A1: ");
//  Serial.print(voltage2);
//  Serial.print(" Signal Voltage (A0 - A1): ");
//  Serial.println(voltage1 - voltage2);
}
