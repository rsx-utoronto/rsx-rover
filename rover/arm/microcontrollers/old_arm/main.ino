//Master Code for CIRC 2022 (currently setup to recieve joystick or manual imputs w no encoders)

#include <Arduino.h>
#include <Stream.h>
#include <Wire.h>

#define ENC_DEBUG 0
#define GRIP_ENCODER_MISSING 1
#define Controller_address 0
#define total_no_of_slaves 3
#define total_no_of_sensors 7

const int slave_address[3] = {1,2,3};

const int sensors_per_slave[3] = {1,2,4};
// 0:gripper, 1:tricep, 2:bisep, 3:wrist, 4:    , 5:bbot, 6:base
int    goal_pos[7] = {0,0,0,0,0,0,0};         // The goal position for each MOTOR
int    reverse[7]  = {1,1,0,0,1,0,1};
int    forewards[7]  = {0,0,1,1,0,1,0};
long int actual_pos[7] = {0,0,0,0,0,0,0}; // The reading of each MOTOR encoder
// Pins for each motor
// h:Base, t:tricep, b:bicep, w:wrist, b1, b2, g
const int dirPin[7] = {12,23,4,6,10,22,8};
const int pwmPin[7] = {13,3,5,7,11,2,9};

// switches and speed limits, error ranges, tic ranges, in ranges.
volatile int limitSwitch[7] = {1,1,1,1,1,1,1};
const int spdLimit[7] = {100,255,255,255,200,200,70};
int cur_pow[7] = {80,255,255,255,200,200,70};
const int error[7] = {20,10,10,20,20,20,10};
const int max_pos[7] = {12700,2075,2600,10000,-5000,-1,-1};
const long int max_in[7] = {-120,-95,-52,-180,-118, -2142857143,0};
const long int min_in[7] = {260,86,56,180,86,2142857143,-0};
// Flags
bool running = true;          // Used for emergency stopping
bool manual_override = false; // Used for the 'm' command

int readable = 0;

void receiveEvent();
void drivers_initilize();
void get_encoder_values();
void move_motors();
void printSensors();
void lim_correct();
void get_commands();

void setup() {     
    Wire.begin(Controller_address); // put your setup code here, to run once:
    Wire.onReceive(receiveEvent);
    Serial.begin(115200);
    //Serial.println("Serial Initialized!!");
    while (!Serial);
    Serial.setTimeout(2); // Set the timeout to 2ms, otherwise parsing can hang for up to a second
    drivers_initilize();
    //Serial.println("Drivers and encoders initialized.");
}

unsigned long last_print = millis();
unsigned long last_override = 0;

void loop() {
    //get_encoder_values();
    get_commands();
    //lim_correct();
    move_motors();
    if(readable>1000){   //fine tune limit to reduce juttering and increase readability 
      printSensors();
      readable=0;
    }else{
      readable++;
    }
}

void get_commands(){
  if (Serial.available()<10 && Serial.available()) {
        char c = Serial.read();
        int v =Serial.parseInt();
        Serial.flush();
        
        if(c == 'f'){
          Serial.print("Command Recieved!! Traveling to Position: ");
          Serial.println(100);
          Serial.println("Moving");
          goal_pos[1] = 100;
          goal_pos[2] = 100;
          cur_pow[1] = 210;
        }else if(c == 'd'){
          Serial.println(100);
          Serial.println("Moving");
          goal_pos[1] = -100;
          goal_pos[2] = -100;
          cur_pow[1] = 220;
        }else if(c == 'g'){
          goal_pos[6] = v ;
          Serial.print("Command Recieved!! Traveling to Position: ");
          Serial.println(goal_pos[6]);
          Serial.println("Moving");
        }else if(c == 't'){
          goal_pos[1] = v ;
          cur_pow[1] = 255;
          Serial.print("Command Recieved!! Traveling to Position: ");
          Serial.println(goal_pos[1]);
          Serial.println("Moving");
        }else if(c == 'b'){
          goal_pos[2] = v ;
          Serial.print("Command Recieved!! Traveling to Position: ");
          Serial.println(goal_pos[2]);
          Serial.println("Moving");
        }else if(c == 'w'){
          goal_pos[3] = v ;
          Serial.print("Command Recieved!! Traveling to Position: ");
          Serial.println(goal_pos[3]);
          Serial.println("Moving");
        }else if(c == 'r'){
          goal_pos[4] += v ;
          goal_pos[5]-= v ;
          Serial.print("Command Recieved!! Traveling to Position: ");
          Serial.println(goal_pos[4]);
          Serial.println("Moving");
        }else if(c == 'x'){
          goal_pos[4] += v ;
          goal_pos[5] += v ;
          Serial.print("Command Recieved!! Traveling to Position: ");
          Serial.println(goal_pos[4]);
          Serial.println("Moving");
        }else if(c == 'h'){
          goal_pos[0] = v ;
          Serial.print("Command Recieved!! Traveling to Position: ");
          Serial.println(goal_pos[0]);
          Serial.println("Moving");
        }else if(c == 's'){
          Serial.println("- - - - - - - - - - - - - - - - - - - - - - - - - - - ");
          Wire.beginTransmission(v);  // send data to slave number [com.substring(1).toInt()]
          Wire.write(' ');
          Wire.endTransmission();
        }else if(c == 'k'){
          for(int i = 0; i<7; i++){
            goal_pos[i]=actual_pos[1];
          }
        }else if(c == 'z'){
          for(int i = 0; i<7; i++){
            goal_pos[i]=0;
          }
        }
    }else if(Serial.available()){
      int controll[7] = {0,0,0,0,0,0,0};
      int pow_state = Serial.parseInt();
      for(int i = 0; i<7; i++){
        controll[i] = Serial.parseInt();
        goal_pos[i] = controll[i]*100;
        if(pow_state>0){
          cur_pow[i] = 255;
        }else{
          cur_pow[i] = spdLimit[i];
        }
      }
      Serial.flush();

      /*
      // used for IK input (as of 2022-08-13)
      int h = Serial.parseInt();
      int t = Serial.parseInt();
      int b = Serial.parseInt();
      int w = Serial.parseInt();
      int top= Serial.parseInt();
      int bot= Serial.parseInt();
      int g = Serial.parseInt();
      Serial.flush();


      goal_pos[0] = h;
      goal_pos[1] = t;
      goal_pos[2] = b;
      goal_pos[3] = w;
      goal_pos[4] = top;
      goal_pos[5] = bot;
      goal_pos[g] = g;
      
      */
      /*
      goal_pos[0] = map(h,min_in[0],max_in[0],0,max_pos[0]);
      goal_pos[1] = map(t,min_in[1],max_in[1],0,max_pos[1]);
      goal_pos[2] = map(b,min_in[2],max_in[2],0,max_pos[2]);
      goal_pos[3] = map(w,min_in[3],max_in[3],0,max_pos[3]);
      goal_pos[4] = map(top,min_in[4],max_in[4],0,max_pos[4]);
      goal_pos[5] = bot;
      */
      /*
      Serial.println(h);
      Serial.println(t);
      Serial.println(b);
      Serial.println(w);
      Serial.println(top);
      Serial.println(bot);
      Serial.println(g);
      */
    }
}

void lim_correct(){
  if(limitSwitch[6] == 0){
    goal_pos[4]= actual_pos[4];
    goal_pos[5]= actual_pos[5] ;
  }

  if(limitSwitch[4] == 0){
    goal_pos[3]= actual_pos[3];
  }

  if(limitSwitch[0] == 0){
    goal_pos[0]= actual_pos[0];
  }
}


void move_motors(){
  int dir = 0;
  int pwm = 0;
  int spd = 0;
  int err = 0;
  for(int i = 0; i<7; i++){
    dir = dirPin[i];
    pwm = int(pwmPin[i]);
    spd = cur_pow[i];
    err = error[i];
    if(goal_pos[i] < actual_pos[i]-err){
      digitalWrite(dir, reverse[i]);
      analogWrite(pwm,spd);
      if(readable>1000){
        Serial.print("Moving Backwards!! Current Position: ");
        Serial.println(actual_pos[i]);
      }
    }else if(goal_pos[i] > actual_pos[i]+err){
      digitalWrite(dir, forewards[i]);
      analogWrite(pwm, spd);
      if(readable>1000){
        Serial.print("Moving Forwards!! Current Position: ");
        Serial.println(actual_pos[i]);
      }
    }else{
      analogWrite(pwm, 0);
    }
    
  }
  
}


void get_encoder_values() {
  Wire.requestFrom(1, 6); // request 6 bytes from slave
  delay(2);
  while (Wire.available()){
    receiveEvent(1);
  }
  Wire.requestFrom(2, 12); // request 6 bytes from slave
  delay(2);
  while (Wire.available()){
    receiveEvent(1);
  }
  Wire.requestFrom(3, 24); // request 6 bytes from slave
  delay(2);
  while (Wire.available()){
    receiveEvent(1);
  }
}

void receiveEvent(int sensor_noo) {
    while (Wire.available()) {
            int sensor_no = int(Wire.read());
            limitSwitch[sensor_no] = int(Wire.read());
            byte a = Wire.read(); 
            byte b = Wire.read();
            byte c = Wire.read();
            byte d = Wire.read();


            actual_pos[sensor_no] = a;
            actual_pos[sensor_no] = (actual_pos[sensor_no] << 8 )| b;
            actual_pos[sensor_no] = (actual_pos[sensor_no] << 8 )| c;
            actual_pos[sensor_no] = (actual_pos[sensor_no] << 8 )| d;  
    }
}

void printSensors(){
  Serial.println("Current Positions:");
  for(int i = 0; i<7; i++){
    Serial.print(actual_pos[i]);
    Serial.print("  ");
  }

  Serial.println("");
  Serial.println("Goal Positions:");
  for(int i = 0; i<7; i++){
    Serial.print(goal_pos[i]);
    Serial.print("  ");
  }
  Serial.println("");
  /*
  Serial.println("Limit Switch Positions:");
  for(int i = 0; i<7; i++){
    Serial.print(limitSwitch[i]);
    Serial.print("  ");
  }
  Serial.println("");
  */
  
  
}

void drivers_initilize() {
    for (int i = 0; i < 7; i++) {
        pinMode(dirPin[i], OUTPUT);
        pinMode(pwmPin[i], OUTPUT);
    } 
}
