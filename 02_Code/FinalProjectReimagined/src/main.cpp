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

#define gyro


#define SERVO_FREQ 50



//motor definitions
// old way to desinate each leg, legacy, but the kinematic model depends on these values.
// DO NOT CHANGE
int aHip = 0; //
int bHip = 3; //
int cHip = 6; //
int dHip = 9; //

//angle variables
double yPreRot= 0, zPreRot = 0;
double yRot= 0, zRot = 0;

//PID setup for gyro
double Kp= 1, Ki=0, Kd =0;
double angleGoal = 0;


class rampLeg{
  private:
  int mhip;
  public:
    ramp hip;
    ramp knee;
    ramp ankle;
    int cycle;

    rampLeg(int mhip){
        hip = mhip;
    }
    bool allDone(){
      if(hip.isFinished() && knee.isFinished() && ankle.isFinished()){
        return true;
      }else{
        return false;
      }
    }
    void setPositions(float Vhip, float Vknee, float Vankle, float timee){
      hip.go(Vhip, timee);
      knee.go(Vknee, timee);
      ankle.go(Vknee, timee);
    }
    void reset(){
      hip.go(0, 250);
      knee.go(0, 250);
      ankle.go(0, 250);
      if (mhip ==0 || mhip == 6){
        cycle = 0;
      }else{
        cycle = 3;
      }
    }
    void update(){
      hip.update();
      knee.update();
      ankle.update();
    }
    void incCycle(){
      cycle++;
      if (cycle > 6){
        cycle = 0;
      }
    }

};

rampLeg a = rampLeg(aHip);
rampLeg b = rampLeg(bHip);
rampLeg c = rampLeg(cHip);
rampLeg d = rampLeg(dHip);

//where in the walk cycle the robot is at. Way to share state across all movements
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


//values for movement
//current /default values 
float testHeight = 150;
float testHeightBACK = 150;
float testLR = 0;
float testFB = 0;

//time for a cycle (in ms)
float timee = 200;

//amount to change values by in a cycle
float backDistance = 80; //(FB)
float upDistance = 80; //xH
float LRDistance = 80; //xLR

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
  //WalkF(-yRot, -zRot);
  WalkLR(yRot, zRot, false);

  // mainKinematics(testHeightBACK, 0, 0, aHip,0,-yRot, -zRot);
  // mainKinematics(testHeightBACK, 0, 0, bHip,0,-yRot, -zRot);
  // mainKinematics(testHeightBACK, 0, 0, cHip,0,-yRot, -zRot);
  // mainKinematics(testHeightBACK, 0, 0, dHip,0,-yRot, -zRot);

  // WalkF(0, 0);



// float a = 150;
// float c = 0;
// float b = -50;
//   mainKinematics(a, b, c, cHip,0,0, 0);
//   mainKinematics(a, b, c, aHip,0,0, 0);
// mainKinematics(a, b, c, dHip,0,0,0);
//  mainKinematics(a, b, c, bHip,0,0, 0);

  //all_90s();



}

