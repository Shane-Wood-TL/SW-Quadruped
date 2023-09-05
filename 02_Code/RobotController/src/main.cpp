#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <LiquidCrystal_I2C.h>

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

//fucntions
void updateMenu(int state);
int incState(int state);
int decState(int state);
float rationalizeJoystick(float value);

//display
LiquidCrystal_I2C lcd(0x27,20,4);

//radio setup
RF24 radio(9, 10);
uint8_t address[][6] = { "1Node", "2Node" };
bool radioNumber = 0;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit
// Used to control whether this node is sending or receiving
bool role = true;  // true = TX role, false = RX role


//joystick inputs
float j1_X_angle, j1_Y_angle, j2_X_angle, j2_Y_angle;
float j1_X_angle_r = 0, j1_Y_angle_r = 0, j2_X_angle_r = 0, j2_Y_angle_r = 0;

//switch / button inputs
int sw1V, sw2V, sw3V, sw4V, sw5V, j1_BV, j2_BV;

//amount of states on the controller (must match robot)
int state;
int maxStates =5;

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
};
PayloadStruct payload;


void setup() {
  Serial.begin(9600);
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


  // begin radio
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    lcd.print("no radio");
    while (1) {}  // hold in infinite loop
  }
   radio.setPALevel(RF24_PA_HIGH);
   radio.stopListening();
}



void loop() {
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
    payload.eStop = true;
  }else{
    payload.eStop = false;
  }
  
  if (sw1V != 1){ //might cause watchdog error, consider rework
      if(j1_X_angle_r >= .25){
        delay(100);
        state = decState(state);
      }else if(j1_X_angle_r <= -.25){
        delay(100);
        state = incState(state);
      }
      payload.state = state;
      updateMenu(state);
      loop(); //cursed but should prevent other options from being changed
  }

  if(sw3V != 1){
    payload.gyro = false;
  }else{
    payload.gyro = true;
  }

  if(sw3V != 4){
    payload.PID = false;
  }else{
    payload.PID = true;
  }
 
 // radio.write(&payload, sizeof(PayloadStruct));
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

void updateMenu(int state){

  lcd.setCursor(0,0);
  lcd.print("Control");
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

  if(payload.gyro == true){
    lcd.setCursor(16,2);
    lcd.print("gyro");
  }
  if(payload.PID == true){
    lcd.setCursor(15,2);
    lcd.print("P");
  }
}

float rationalizeJoystick(float value){
  ////874 max, 443 = mid, 0 min
  value = map(value,0,875,-1,1);
  return value;
}