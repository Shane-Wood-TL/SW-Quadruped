#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <SPI.h>

#include <Ramp.h>
#include <Adafruit_PWMServoDriver.h>
#include <utility/imumaths.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <PID_v1.h> 

#include<externFunctions.h>


//uint8_t controller[x]; // x = the amount of things being transferred (starting at 1)
//2 or 3? joysticks
//x joystick1
//x joystick2
//y joystick1
//y joystick2
//button j1
//button j2
//swtich (4?)
  //1 for gyro
  //2 to switch between turn and walk
  //3 auton
  //4 stop
//buttons
  //sit
  //down

//controller will have display with up/down and confirm buttons
//only mode will be sent




//state 0: standing at 150mm, gyro off
//state 1: standing at 150mm, gyro on
//state 2: kinematic input, manual angle
//state 3: kinematic input, gyro
//state 4: walking forward with set measurement
//state 5: turn in place with set measurement
//state 6: user control, gyro off
//state 7: user control gyro on
//possible state 8: add a way to adjust the PID values to better tune it

//values for movement
//current /default values
float testHeight = 150;
float testHeightBACK = 170;
float testLR = 0;
float testFB = 0;

//time for a cycle (in ms)
float timee = 100;

//amount to change values by in a cycle
float backDistance = 50; //(FB)
float upDistance = -50; //xH
float LRDistance =100; //xLR


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
#ifdef gyro
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
#endif
sensors_event_t event;

PID yPID(&yPreRot, &yRot, &angleGoal, Kp, Ki, Kd, DIRECT);
PID zPID(&zPreRot, &zRot, &angleGoal, Kp, Ki, Kd, DIRECT);

//PCA9685 setup
#define SERVO_FREQ 50
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);

void setup() {
  Serial.begin(115200);
  Serial.println("alive");
  Wire.begin(17,15); //SDA, SCL
  Serial.println("IC2 alive");

  #ifdef gyro
    bno.begin();
    //set up gyro
    Serial.println("gyro started");
    if(!bno.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while(1);
    }
    //bno.setExtCrystalUse(true);
    delay(1000); 
    Serial.println("gyroalive");
    yPID.SetOutputLimits(-45.0,45.0);
    zPID.SetOutputLimits(-45.0,45.0);

        //set the pid mode
    yPID.SetMode(AUTOMATIC);
    zPID.SetMode(AUTOMATIC);
    Serial.println("PID alive");
  #endif
  
  //Set up PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);  

  pwm1.begin();
  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  //where in the walk cycle the robot is at. Way to share state across all movements
  aLeg.setCycle(0);
  bLeg.setCycle(3);
  cLeg.setCycle(3);
  dLeg.setCycle(0);

  mainKinematics(testHeight, 0, 0, aHip,0,0,0);
  delay(200);
  mainKinematics(testHeight, 0, 0, cHip,0,0,0);
  mainKinematics(testHeight, 0, 0, bHip,0,0,0);
  delay(200);
  mainKinematics(testHeight, 0, 0, dHip,0,0,0);

  aLeg.reset();
  bLeg.reset();
  cLeg.reset();
  dLeg.reset();
}

void loop() {
  
  #ifdef gyro  
    bno.getEvent(&event);
    delay(5);
    yPreRot = event.orientation.y;
    zPreRot = event.orientation.z;
   yPID.Compute();
    zPID.Compute();
  #endif
  #ifndef gyro
    yRot =0;
    zRot = 0;
  #endif

turn(yRot, zRot, false);
//WalkF(yPreRot,zPreRot, true);


int state = 0;


//state 0: standing at 150mm, gyro off
//state 1: standing at 150mm, gyro on
//state 2: kinematic input, manual angle
//state 3: kinematic input, gyro
//state 4: walking forward with set measurement
//state 5: turn in place with set measurement
//state 6: user control, gyro off
//state 7: user control gyro on
//possible state 8: add a way to adjust the PID values to better tune it
switch(state){
  case(0):{
    mainKinematics(150,0,0,aHip,0,0,0);
    mainKinematics(150,0,0,bHip,0,0,0);
    mainKinematics(150,0,0,cHip,0,0,0);
    mainKinematics(150,0,0,dHip,0,0,0);
  }
  case(1):{
    mainKinematics(150,0,0,aHip,0,yRot,zRot);
    mainKinematics(150,0,0,bHip,0,yRot,zRot);
    mainKinematics(150,0,0,cHip,0,yRot,zRot);
    mainKinematics(150,0,0,dHip,0,yRot,zRot);

  }
  case(2):{
    
    // mainKinematics(,0,0,aHip,0,,);
    // mainKinematics(,0,0,bHip,0,,);
    // mainKinematics(,0,0,cHip,0,,);
    // mainKinematics(,0,0,dHip,0,,);
  }
  case(3):{
    // mainKinematics(,0,0,aHip,0,yRot,zRot);
    // mainKinematics(,0,0,bHip,0,yRot,zRot);
    // mainKinematics(,0,0,cHip,0,yRot,zRot);
    // mainKinematics(,0,0,dHip,0,yRot,zRot);
  }
  case(4):{
    WalkF(yRot,zRot, true);
  }
  case(5):{
    turn(yRot, zRot, true);
  }
  case(6):{
    //not sure how to implement this yet, as the cycles will need to break at times and then
    //change back into "normal"
  }
  case(7):{
    //same as above ^ not sure, however they will have gyro enabled
  }

}
}

