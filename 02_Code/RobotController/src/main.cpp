#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <LiquidCrystal_I2C.h>
#include "printf.h"

//joystick pins
#define j1_X A0
#define j1_Y A1
#define j2_X A2
#define j2_Y A3

//joystick button pins
#define j1_B 7
#define j2_B 8


//swtich pins
#define sw1 2
#define sw2 3
#define sw3 4
#define sw4 5
#define sw5 6

//the data sent with the radio
struct PayloadStruct {
  bool eStop; //sw2
  int state;
  bool gyro;
  bool PID;
  float j1_x;
  float j1_y;
  float j2_x;
  float j2_y;
  bool j1_b;
  bool j2_b;
};
PayloadStruct payload;

//fucntions
void updateMenu(int state, PayloadStruct payload);
int incState(int state);
int decState(int state);
float rationalizeJoystick(float value);

//display
LiquidCrystal_I2C lcd(0x27,20,4);

//radio setup
RF24 radio(9, 10);
const byte slaveAddress[5] = {'R','x','A','A','A'};


//joystick inputs
float j1_X_angle, j1_Y_angle, j2_X_angle, j2_Y_angle;
float j1_X_angle_r = 0, j1_Y_angle_r = 0, j2_X_angle_r = 0, j2_Y_angle_r = 0;

//switch / button inputs
int sw1V, sw2V, sw3V, sw4V, sw5V, j1_BV, j2_BV;

//amount of states on the controller (must match robot)
int state;
int maxStates =5;




void setup() {
  Serial.begin(9600);
  printf_begin();
  Serial.print("alive");
  //set up display
  lcd.init();           
  lcd.backlight();
  lcd.clear();
  Serial.print("here2");
  //set all button/switches ;as input pullup (no resistor is used, buttons are tied to gnd)
  pinMode(sw1, INPUT_PULLUP);
  pinMode(sw2, INPUT_PULLUP);
  pinMode(sw3, INPUT_PULLUP);
  pinMode(sw4, INPUT_PULLUP);
  pinMode(sw5, INPUT_PULLUP);
  pinMode(j1_B, INPUT_PULLUP);
  pinMode(j2_B, INPUT_PULLUP);

  radio.begin();
  // begin radio
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    lcd.print("no radio");
    while (1) {}  // hold in infinite loop
  }
  
   radio.setPALevel(RF24_PA_MIN);
   radio.setDataRate( RF24_250KBPS );
    radio.setRetries(3,5); // delay, count
   radio.openWritingPipe(slaveAddress);

   updateMenu(state, payload);
   payload.j1_x = 5;
}



void loop() {
  //radio.printPrettyDetails();
  //update all inputs
  j1_X_angle = analogRead(j1_X); 
  j1_Y_angle = analogRead(j1_Y);
  j2_X_angle = analogRead(j2_X);
  j2_Y_angle = analogRead(j2_Y);
  j1_BV = digitalRead(j1_B);
  j2_BV = digitalRead(j2_B);

  j1_X_angle_r = rationalizeJoystick(j1_X_angle);
  j1_Y_angle_r = rationalizeJoystick(j1_Y_angle);
  j2_X_angle_r = rationalizeJoystick(j2_X_angle);
  j2_Y_angle_r = rationalizeJoystick(j2_Y_angle);
  
  //button on joystick = turn in place one or other direction
  // i sw1 = menu mode
  // i sw2 = move / estop
  // i sw3 = gyro
  // sw4 = PID on / off
  // sw5 = future / na
  sw1V = digitalRead(sw1);
  sw2V = digitalRead(sw2);
  sw3V = digitalRead(sw3);
  sw4V = digitalRead(sw4);
  sw5V = digitalRead(sw5);

  
  if(sw2V != 1){
    if(payload.eStop != true){
      payload.eStop = true;
      updateMenu(state, payload);
    }
  }else{
    if(payload.eStop != false){
      payload.eStop = false;
      updateMenu(state, payload);
    }
    
  }

  if(j1_B != 1){ //need to convert to toggle
    payload.j1_b = true;
  }else{
    payload.j1_b = false;
  }

   if(j2_B != 1){
    payload.j2_b = true;
  }else{
    payload.j2_b = false;
  }

  if (sw1V != 1){
    int oldState = state;
      if(j1_X_angle_r >= .25){
        state = decState(state);
        delay(100);
      }else if(j1_X_angle_r <= -.25){
        state = incState(state);
        delay(100);
      }
     
      if (state != oldState){
        payload.state = state;
        updateMenu(state, payload);
        
      }
  }

  if(sw3V != 1){
    if(payload.gyro != false){
      payload.gyro = false;
      updateMenu(state, payload);
    }
  }else{
    if(payload.gyro != true){
      payload.gyro = true;
      updateMenu(state, payload);
    }
  }

  if(sw4V != 1){
    if(payload.PID != false){
      payload.PID = false;
      updateMenu(state, payload);
    }
  }else{
    if(payload.PID != true){
      payload.PID = true;
      updateMenu(state, payload);
    }
  }
 

  payload.state = state;
  payload.j1_x = j1_X_angle_r;
  payload.j1_y = j1_Y_angle_r;
  payload.j2_x = j2_X_angle_r;
  payload.j2_y = j2_Y_angle_r;

  Serial.print(radio.write(&payload, sizeof(PayloadStruct)));
  

  // Serial.print(sw1V);
  // Serial.print(sw2V);
  // Serial.print(sw3V);
  // Serial.print(sw4V);
  // Serial.print(sw5V);
  // Serial.println();
}


int incState(int state){
  state++;
  if (state >= maxStates){
    state = 0;
  }
  return state;
}

int decState(int state){
  state--;
  if (state < 0){
    state = maxStates-1;
  }
  return state;
}

void updateMenu(int state, PayloadStruct payload){
  lcd.clear();
  if(payload.gyro == true){
    lcd.setCursor(10,1);
    lcd.print("gyro");
  }
  if(payload.PID == true){
    lcd.setCursor(15,1);
    lcd.print("P");
  }

  if(payload.eStop == true){
    lcd.setCursor(1,1);
      lcd.print("STOP");
  }else{
      lcd.setCursor(1,1);
      lcd.print("   ");
  }
  lcd.setCursor(0,0);
  lcd.print("Control: ");
  switch (state)
  {
  case 0:{
    lcd.print("Stand");
    break;
  }
  case 1:{
     lcd.print("IK mode");
     break;
  }
  case 2:{
    lcd.print("FWalk");
    break;
  }
  case 3:{
    lcd.print("Fturn");
    break;
  }
  case 4:{
    lcd.print("User");
    break;
  }
  default:{
    break;
  }
  } 
}

float rationalizeJoystick(float value){
  ////874 max, 443 = mid, 0 min
  value = map(value,0,875,-1,1);
  return value;
}