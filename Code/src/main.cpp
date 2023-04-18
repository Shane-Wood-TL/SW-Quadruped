#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <SPI.h>

#include <Adafruit_PWMServoDriver.h>

#include <utility/imumaths.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include <Ramp.h>
#include <PID_v1.h>  

#include<externFunctions.h>


void walk(int& Cycle, ramp &FB, ramp &Height, float timee, float backDistance, float upDistance);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50

//motor definitions
const int aHip = 0; //
const int bHip = 3; //
const int cHip = 6; //
const int dHip = 9; //


float testHeight = 120;
float testHeightBACK = 120;
float testLR = 0;
float testFB = 0;

ramp aHeight;
ramp bHeight;
ramp cHeight;
ramp dHeight;

ramp aFB;
ramp bFB;
ramp cFB;
ramp dFB;

//where in the walk cycle the robot is at
int aCycle = 0;
int bCycle = 3;
int cCycle = 0;
int dCycle = 3;

//6, 4, 1, 3

//angle variables
double yPreRot= 0, zPreRot = 0;
double yRot= 0, zRot = 0;
double Kp= .05, Ki=0, Kd =0;
double angleGoal = 0;

PID yPID(&yPreRot, &yRot, &angleGoal, Kp, Ki, Kd, DIRECT);
PID zPID(&zPreRot, &zRot, &angleGoal, Kp, Ki, Kd, DIRECT);


  // float timee = 250;
  // float backDistance = 50;
  // float upDistance = 30;

  float timee = 50;
  float backDistance = 40;
  float upDistance = 20;


//enables or disables the use of the gyro
bool useGyro = false;



void setup() {

  Serial.begin(115200);
  Wire.begin(17,15); //SDA, SCL

  //Set up PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);  

  mainKinematics(120, 0, 0, aHip,0,0,0);
  mainKinematics(120, 0, 0, cHip,0,0,0);
  mainKinematics(120, 0, 0, bHip,0,0,0);
  mainKinematics(120, 0, 0, dHip,0,0,0);
  delay(2000);

  if (useGyro == true){
    bno.begin();
    //set up gyro
    if(!bno.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while(1);
    }
    bno.setExtCrystalUse(true);
    delay(1000); 
    yPID.SetOutputLimits(-45.0,45.0);
    zPID.SetOutputLimits(-45.0,45.0);
    }
  //give the ramps some inital values

  aHeight.go(0,0);
  aFB.go(0,0);

  bHeight.go(0,0);
  bFB.go(0,0);

  cHeight.go(0,0);
  cFB.go(0,0);

  dHeight.go(0,0);
  dFB.go(0,0);

  //set the pid mode
  yPID.SetMode(AUTOMATIC);
  zPID.SetMode(AUTOMATIC);

}

void loop() {

  
  if(useGyro){
    sensors_event_t event;
    bno.getEvent(&event);
    yPreRot = event.orientation.y;
    zPreRot = event.orientation.z;

    //yRot = event.orientation.y;
    //zRot = event.orientation.z;

    //angleGoaly = yPreRot;
    //angleGoalz = zPreRot;

    yPID.Compute();
    zPID.Compute();

    // Serial.print("Goal: ");
    // Serial.print(yPreRot);
    // Serial.print(" At: ");
    // Serial.println(yRot);
  }else{
    yRot =0;
    zRot = 0;
  }
  
  Serial.print("/*");
  Serial.print(0);
  Serial.print(",");
  Serial.print(yRot);
  Serial.print(",");
  Serial.print(zRot);
  Serial.print(",");
  Serial.print(aCycle);
  Serial.print(",");
  Serial.print(bCycle);
  Serial.print(",");
  Serial.print(cCycle);
  Serial.print(",");
  Serial.print(dCycle);
  Serial.print(",");
// mainKinematics(75, 0, testLR, aHip,0,0,0);
// mainKinematics(testHeight, 0, testLR, bHip,0,yRot,zRot);
// mainKinematics(75, 0, testLR, cHip,0,0,0);
// mainKinematics(testHeight, 0, testLR, dHip,0,yRot,zRot);
   

  //void walk(int &Cycle, ramp &FB, ramp &Height, float time, float backDistance, float upDistance);
  if(aFB.isFinished() && bFB.isFinished() && cFB.isFinished() && dFB.isFinished()){
    walk(bCycle, bFB, bHeight, timee, backDistance, upDistance);
    walk(cCycle, cFB, cHeight, timee, backDistance, upDistance);
    walk(dCycle, dFB, dHeight, timee, backDistance, upDistance);
    walk(aCycle, aFB, aHeight, timee, backDistance, upDistance);
  }
// // aHeight.go(upDistance,0);
// // dHeight.go(upDistance,0);


  aFB.update();
  aHeight.update();
  if(aCycle == 4 || aCycle == 5 || aCycle == 6){
    mainKinematics(testHeight-aHeight.getValue(), -aFB.getValue(), testLR, aHip,0,0,0);
  }else{
    mainKinematics(testHeight-aHeight.getValue(), -aFB.getValue(), testLR, aHip,0,yRot,zRot);
  }
  

  cFB.update();
  cHeight.update();
  if(cCycle == 4 || cCycle == 5 || cCycle == 6){
    mainKinematics(testHeightBACK-cHeight.getValue(), -cFB.getValue(), testLR, cHip,0,0, 0);
  }else{
    mainKinematics(testHeightBACK-cHeight.getValue(), -cFB.getValue(), testLR, cHip,0,yRot,zRot);
  }
  

  bFB.update();
  bHeight.update();
    if(bCycle == 4 || bCycle == 5 || bCycle == 6){
    mainKinematics(testHeight-bHeight.getValue(), -bFB.getValue(), testLR, bHip,0,0,0);
  }else{
    mainKinematics(testHeight-bHeight.getValue(), -bFB.getValue(), testLR, bHip,0,yRot,zRot);
  }


  dFB.update();
  dHeight.update();
    if(dCycle == 6 || dCycle == 4 || dCycle == 5){
    mainKinematics(testHeightBACK-dHeight.getValue(), -dFB.getValue(), testLR, dHip,0,0,0);
  }else{
   mainKinematics(testHeightBACK-dHeight.getValue(), -dFB.getValue(), testLR, dHip,0,yRot,zRot);
  }

  // if (Serial.available()){
  //   upDistance = Serial.parseFloat();
  //   backDistance = Serial.parseFloat();
  //   timee = Serial.parseFloat();
  // }
  Serial.println("*/");
}