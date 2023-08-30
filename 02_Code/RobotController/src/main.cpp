#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <LiquidCrystal_I2C.h>

void updateMenu(int state, bool gyro);
const int j1_X = A0;
const int j1_Y = A1;
const int j2_X = A2;
const int j2_Y = A3;

const int CE = 9;
const int CSN = 10;
const int SCKP = 13;
const int MOSIP = 11;
const int MISOP = 12;

const int sw1 = 2;
const int sw2 = 3;
const int sw3 = 4;
const int sw4 = 5;
const int sw5 = 6;

const int j1_B = 7;
const int j2_B = 8;

LiquidCrystal_I2C lcd(0x27,20,4);


float j1_X_angle, j1_Y_angle, j2_X_angle, j2_Y_angle;
float j1_X_angle_s = 0, j1_Y_angle_s = 0, j2_X_angle_s = 0, j2_Y_angle_s = 0;

int sw1V, sw2V, sw3V, sw4V, sw5V, j1_BV, j2_BV;
const float smoothingFactor = 0.1; 
int state;

RF24 radio(10, 9);

uint8_t address[][6] = { "1Node", "2Node" };

bool radioNumber = 0;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Used to control whether this node is sending or receiving
bool role = true;  // true = TX role, false = RX role
bool gyro = true;

struct PayloadStruct {
  bool eStop; //sw2
  int state;
  bool gyro;
  float j1_x;
  float j1_y;
  float j2_x;
  float j2_y;
};
PayloadStruct payload;


void setup() {
  Serial.begin(9600);


  lcd.init();                      // initialize the lcd 
  lcd.backlight();


  pinMode(sw1, INPUT_PULLUP);
  pinMode(sw2, INPUT_PULLUP);
  pinMode(sw3, INPUT_PULLUP);
  pinMode(sw4, INPUT_PULLUP);
   pinMode(sw5, INPUT_PULLUP);

  pinMode(j1_B, INPUT_PULLUP);
  pinMode(j2_B, INPUT_PULLUP);


  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }
  Serial.println("uh");
   radio.setPALevel(RF24_PA_HIGH);
   radio.stopListening();
}

void loop() {
  j1_X_angle = analogRead(j1_X); //874 max, 443 = mid, 0 min
  j1_Y_angle = analogRead(j1_Y);
  j2_X_angle = analogRead(j2_X);
  j2_Y_angle = analogRead(j2_Y);


  j1_BV = digitalRead(j1_B);
  j2_BV = digitalRead(j2_B);

  sw1V = digitalRead(sw1);
  sw2V = digitalRead(sw2);
  sw3V = digitalRead(sw3);
  sw4V = digitalRead(sw4);
  sw5V = digitalRead(sw5);

  j1_X_angle_s = (smoothingFactor * j1_X_angle) + ((1 - smoothingFactor) * j1_X_angle_s);
  j1_Y_angle_s = (smoothingFactor * j1_Y_angle) + ((1 - smoothingFactor) * j1_Y_angle_s);
  j2_X_angle_s = (smoothingFactor * j2_X_angle) + ((1 - smoothingFactor) * j2_X_angle_s);
  j2_Y_angle_s = (smoothingFactor * j2_Y_angle) + ((1 - smoothingFactor) * j2_Y_angle_s);

  if(sw2V != 1){
    payload.eStop = true;
  }

  //button on joystick = turn in place one or other direction
  // sw1 = menu mode
  // sw2 = move / estop (chanage)
  // sw3 = gyro
  // sw4 = sit
  // sw5 = 

  

}



void updateMenu(int state, bool gyro){
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

  if(gyro == true){
    lcd.setCursor(16,2);
    lcd.print(gyro);
  }
}