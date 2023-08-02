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




#define SERVO_FREQ 50

#define gyro

//motor definitions
int aHip = 0; //do not update (not correct, but change is not needed)
int bHip = 3; //
int cHip = 6; //
int dHip = 9; //

//angle variables
double yPreRot= 0, zPreRot = 0;
double yRot= 0, zRot = 0;
double Kp= 1, Ki=0, Kd =0;
double angleGoal = 0;


float testHeight = 150;
float testHeightBACK = 150;
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

ramp aLR;
ramp bLR;
ramp cLR;
ramp dLR;

//where in the walk cycle the robot is at
int aCycle = 0;
int bCycle = 3;
int cCycle = 3;
int dCycle = 0;

#ifdef gyro
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
#endif

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);


PID yPID(&yPreRot, &yRot, &angleGoal, Kp, Ki, Kd, DIRECT);
PID zPID(&zPreRot, &zRot, &angleGoal, Kp, Ki, Kd, DIRECT);

sensors_event_t event;

float timee = 125;
float backDistance = 60;
float upDistance = 25;
float LRDistance = 0;

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


  mainKinematics(120, 0, 0, aHip,0,0,0);
  delay(200);
  mainKinematics(120, 0, 0, cHip,0,0,0);
  mainKinematics(120, 0, 0, bHip,0,0,0);
  delay(200);
  mainKinematics(120, 0, 0, dHip,0,0,0);

  
  //give the ramps some inital values

  resetAll();
  


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
  Serial.print(yRot);
  Serial.println(zRot);

  //turner(yRot, zRot);
  //WalkF(yRot, zRot);
  //WalkLR(yRot, zRot, false);

  // mainKinematics(testHeightBACK, 0, 0, aHip,0,0, 0);
  // mainKinematics(testHeightBACK, 0, 0, bHip,0,0, 0);
  // mainKinematics(testHeightBACK, 0, 0, cHip,0,0, 0);
  // mainKinematics(testHeightBACK, 0, 0, dHip,0,0, 0);

  WalkF(0, 0);



// float a = 150;
// float c = 0;
// float b = -50;
//   mainKinematics(a, b, c, cHip,0,0, 0);
//   mainKinematics(a, b, c, aHip,0,0, 0);
// mainKinematics(a, b, c, dHip,0,0,0);
//  mainKinematics(a, b, c, bHip,0,0, 0);

  //all_90s();



}

