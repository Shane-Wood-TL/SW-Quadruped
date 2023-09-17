#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <math.h>
#include <Wire.h>
#include <Ramp.h>
#include <Adafruit_PWMServoDriver.h>
#include <utility/imumaths.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <PID_v1.h> 
#include<externFunctions.h>


#define Ugyro

//SCK 18
//MOSI 8
//MISO 10
//CE 16
//CSN 9


//values for non controlled turn
//current /default values
const float testHeightT = 150;
const float testHeightBACKT = 150;
const float testLRT = 0;
const float testFBT = 0;

//time for a cycle (in ms)
const float timeeT = 100;

//amount to change values by in a cycle
const float backDistanceT = 0; //(FB)
const float upDistanceT = -50; //xH
const float LRDistanceT =100; //xLR


//values for non controlled walk
const float testHeightW = 150;
const float testHeightBACKW = 150;
const float testLRW = 0;
const float testFBW = 0;

//time for a cycle (in ms)
const float timeeW = 500;

//amount to change values by in a cycle
const float backDistanceW = -50; //(FB)
const float upDistanceW = -50; //xH
const float LRDistanceW =0; //xLR



//motor definitions
// old way to desinate each leg, legacy, but the kinematic model depends on these values.
// DO NOT CHANGE
int aHip = 0; //
int bHip = 3; //
int cHip = 6; //
int dHip = 9; //

rampLeg aLeg(aHip);
rampLeg bLeg(bHip);
rampLeg cLeg(cHip);
rampLeg dLeg(dHip);

//angle variables
double yPreRot= 0, zPreRot = 0;
double yRot= 0, zRot = 0;

//PID setup for gyro
double Kp= .25, Ki=.01, Kd =0;
double angleGoal = 0;


 //soon to be deprecated, as the gyro will need to be init everytime to
 //allow switching of modes that have it enabled
#ifdef Ugyro
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
#endif
sensors_event_t event;

PID yPID(&yPreRot, &yRot, &angleGoal, Kp, Ki, Kd, DIRECT);
PID zPID(&zPreRot, &zRot, &angleGoal, Kp, Ki, Kd, DIRECT);

//PCA9685 setup
#define SERVO_FREQ 50
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);


int oldState = 0;
//SPI.begin(16, 8, 10, 16);
struct PayloadStruct {
  uint8_t eStop; //sw2
  uint8_t state;
  uint8_t gyro;
  uint8_t PID;
  int16_t j1_x;
  int16_t j1_y;
  int16_t j2_x;
  int16_t j2_y;
  uint8_t j1_b;
  uint8_t j2_b;
};
PayloadStruct payload;

const byte thisSlaveAddress[5] = {'R','x','A','A','A'};
RF24 radio(16,9);

bool newData = false;
