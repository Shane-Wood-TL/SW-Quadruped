#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <utility/imumaths.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include <Ramp.h>  

#include<externFunctions.h>

void selfLevel();


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);


#define SERVO_FREQ 50



//motor definitions
const int aHip = 0; //

const int bHip = 3; //

const int cHip = 6; //

const int dHip = 9; //

float xAngle;
float yAngle;
float zAngle;



float testHeight = 136;
float testLR = 0;
float testFB = 0;

ramp back;
ramp up;
ramp offback;
ramp offup;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
   Wire.begin(17,15); //SDA, SCL
   //Wire.begin();
   

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);
 delay(1000);  


 pwm.begin();
   pwm.setOscillatorFrequency(27000000);
   pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
   delay(10);  

 

}

void loop() {

  // mainKinematics(125, -20, testLR,aHip,0,0,0);
  // mainKinematics(125, -20, testLR,cHip,0,0,0);

  // mainKinematics(100, 0, testLR,bHip,0,0,0);
  // mainKinematics(100, 0, testLR,dHip,0,0,0);
   //delay(200);

  // mainKinematics(100, 0, testLR, aHip,0,0,0);
  // mainKinematics(100, 0, testLR,cHip,0,0,0);
  // mainKinematics(125, -20, testLR,bHip,0,0,0);
  //   mainKinematics(125, -20, testLR,dHip,0,0,0);
  // delay(200);

  sensors_event_t event;
  bno.getEvent(&event);

  Serial.print("X: ");
  Serial.print(event.orientation.x, 0);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 0);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 0);
  Serial.println("");
  //selfLevel();
float yRot= 0, zRot = 0;
// // event.orientation.y event.orientation.z
back.go(75,500);
offback.go(25, 500);
while(back.isFinished() == false){
  back.update();
  offback.update();
  mainKinematics(testHeight, -back.getValue(), testLR, aHip,0,yRot,zRot);
  mainKinematics(testHeight, -back.getValue(), testLR, cHip,0,yRot,zRot);

  mainKinematics(testHeight, -offback.getValue(), testLR, bHip,0,yRot,zRot);
  mainKinematics(testHeight, -offback.getValue(), testLR, dHip,0,yRot,zRot);
}

back.go(0,500);
up.go(30, 500);
offback.go(50, 500);

while(back.isFinished() == false){
  back.update();
  up.update();
  offback.update();

  mainKinematics(testHeight-up.getValue(), -back.getValue(), testLR, aHip,0,yRot,zRot);
  mainKinematics(testHeight-up.getValue(), -back.getValue(), testLR, cHip,0,yRot,zRot);

    mainKinematics(testHeight, -offback.getValue(), testLR, bHip,0,yRot,zRot);
  mainKinematics(testHeight, -offback.getValue(), testLR, dHip,0,yRot,zRot);
}
up.go(0, 100);
offback.go(75, 100);
while(up.isFinished() == false){
  back.update();
  up.update();
  offback.update();

  mainKinematics(testHeight-up.getValue(), -back.getValue(), testLR, aHip,0,yRot,zRot);
  mainKinematics(testHeight-up.getValue(), -back.getValue(), testLR, cHip,0,yRot,zRot);

  mainKinematics(testHeight-offup.getValue(), -offback.getValue(), testLR, bHip,0,yRot,zRot);
  mainKinematics(testHeight-offup.getValue(), -offback.getValue(), testLR, dHip,0,yRot,zRot);

}
delay(100);
offback.go(0,500);
offup.go(30, 500);
while(offback.isFinished() == false){
  offback.update();
  offup.update();
  mainKinematics(testHeight-offup.getValue(), -offback.getValue(), testLR, bHip,0,yRot,zRot);
  mainKinematics(testHeight-offup.getValue(), -offback.getValue(), testLR, dHip,0,yRot,zRot);
}
offup.go(0, 100);
while(offup.isFinished() == false){
  offback.update();
  offup.update();
  mainKinematics(testHeight-offup.getValue(), -offback.getValue(), testLR, bHip,0,yRot,zRot);
  mainKinematics(testHeight-offup.getValue(), -offback.getValue(), testLR, dHip,0,yRot,zRot);
}

delay(100);



//selfLevel();
  // mainKinematics(testHeight, testFB, testLR, aHip,0,-(event.orientation.y),event.orientation.z);
  //   mainKinematics(testHeight, testFB, testLR, bHip,0,event.orientation.y,event.orientation.z);
  //   mainKinematics(testHeight, testFB, testLR, cHip,0,event.orientation.y,-(event.orientation.z));
  //   mainKinematics(testHeight, testFB, testLR, dHip,0,-(event.orientation.y),-(event.orientation.z));
    //       mainKinematics(testHeight, testFB, testLR, aHip,0,0,0);
    // mainKinematics(testHeight, testFB, testLR, bHip,0,0,0);
    // mainKinematics(testHeight, testFB, testLR, cHip,0,0,0);
    // mainKinematics(testHeight, testFB, testLR, dHip,0,0,0);
  
//   int moveToHeight = 85;
//   for(int i = 100; i>moveToHeight; i--){
//     mainKinematics(10+i,(i-100), testLR, aHip,0,0,0);
//     mainKinematics(i, (i-100), testLR, cHip,0,0,0);
//     mainKinematics(10+100, 0, testLR, bHip,0,yRot,zRot);
//   mainKinematics(100, 0, testLR, dHip,0,yRot,zRot);
//   //delay(1);

//   }
// mainKinematics(10+90,0, testLR, aHip,0,0,0);
//     mainKinematics(90, 0, testLR, cHip,0,0,0);
// delay(1);
//   for(int i = 100; i>moveToHeight; i--){
//     mainKinematics(10+i, (i-100), testLR, bHip,0,0,0);
//     mainKinematics((i), (i-100), testLR, dHip,0,0,0);
//     mainKinematics(10+100, 0, testLR, aHip,0,yRot,zRot);
//   mainKinematics(100, 0, testLR, cHip,0,yRot,zRot);
//   //delay(1);
//   }
// mainKinematics(90,0, testLR, dHip,0,0,0);
//     mainKinematics(10+90, 0, testLR, bHip,0,0,0);
//     delay(1);
}
   
   



