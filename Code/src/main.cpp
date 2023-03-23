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



Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50

//motor definitions
const int aHip = 0; //
const int bHip = 3; //
const int cHip = 6; //
const int dHip = 9; //


float testHeight = 136;
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
int bCycle = 2;
int cCycle = 0;
int dCycle = 2;

//angle variables
double yPreRot= 0, zPreRot = 0;
double yRot= 0, zRot = 0;
double Kp=2, Ki=5, Kd =1;
double angleGoal = 0;
PID yPID(&yPreRot, &yRot, &angleGoal, Kp, Ki, Kd, DIRECT);
PID zPID(&zPreRot, &zRot, &angleGoal, Kp, Ki, Kd, DIRECT);

//enables or disables the use of the gyro
bool useGyro = true;
  
void setup() {

  Serial.begin(9600);
  Wire.begin(17,15); //SDA, SCL

  //Set up PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);  

  
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
  
    yPID.Compute();
    zPID.Compute();
  }else{
    yRot =0;
    zRot = 0;
  }
   
  float time = 200;
  float backDistance = 20;
  float upDistance = 30;

  if(aCycle == 0 && (!aFB.isRunning())){
    aFB.go(backDistance,time);
    aCycle = 1;
  }else if (aCycle == 1 && aFB.isFinished())
  {
    aHeight.go(upDistance,time);
    aFB.go(0,time);
    aCycle = 2;
  }else if (aCycle == 2 && aFB.isFinished())
  {
    aHeight.go(0,time);
    aFB.go(0,time);
    aCycle = 0;
  }

  if(cCycle == 0 && (!cFB.isRunning())){
    cFB.go(backDistance,time);
    cCycle = 1;
  }else if (cCycle == 1 && cFB.isFinished())
  {
    cHeight.go(upDistance,time);
    cFB.go(0,time);
    cCycle = 2;
  }else if (cCycle == 2 && cFB.isFinished())
  {
    cHeight.go(0,time);
    cFB.go(0,time);
    cCycle = 0;
  }

  if(bCycle == 0 && (!bFB.isRunning())){
    bFB.go(backDistance,time);
    bCycle = 1;
  }else if (bCycle == 1 && bFB.isFinished())
  {
    bHeight.go(upDistance,time);
    bFB.go(0,time);
    bCycle = 2;
  }else if (bCycle == 2 && bFB.isFinished())
  {
    bHeight.go(0,time);
    bFB.go(0,time);
    bCycle = 0;
  }

  if(dCycle == 0 && (!dFB.isRunning())){
    dFB.go(backDistance,time);
    dCycle = 1;
  }else if (dCycle == 1 && dFB.isFinished())
  {
    dHeight.go(upDistance,time);
    dFB.go(0,time);
    dCycle = 2;
  }else if (dCycle == 2 && dFB.isFinished())
{
  dHeight.go(0,time);
  dFB.go(0,time);
  dCycle = 0;
}



  aFB.update();
  aHeight.update();
  mainKinematics(testHeight-aHeight.getValue(), -aFB.getValue(), testLR, aHip,0,yRot,zRot);

  cFB.update();
  cHeight.update();
  mainKinematics(testHeight-cHeight.getValue(), -cFB.getValue(), testLR, cHip,0,yRot,zRot);

  bFB.update();
  bHeight.update();
  mainKinematics(testHeight-bHeight.getValue(), -bFB.getValue(), testLR, bHip,0,yRot,zRot);

  dFB.update();
  dHeight.update();
  mainKinematics(testHeight-dHeight.getValue(), -dFB.getValue(), testLR, dHip,0,yRot,zRot);


  delay(10);
  bool endWalkCycle = false;
  if(endWalkCycle){
    aHeight.go(0, time);
    bHeight.go(0, time);
    cHeight.go(0, time);
    dHeight.go(0, time);

    aFB.go(0,time);
    bFB.go(0,time);
    cFB.go(0,time);
    dFB.go(0,time);
  }

}
   
   



