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

#define Ugyro
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

void getData();
void showData();

char dataReceived[10]; // this must match dataToSend in the TX
bool newData = false;



void setup() {
  
Serial.begin(9600);
Wire.begin(17,15); //SDA, SCL
  SPI.begin(18, 8, 10);

  Serial.println("SimpleRx Starting");
    radio.begin();
    radio.setDataRate( RF24_250KBPS );
    radio.openReadingPipe(1, thisSlaveAddress);
    radio.startListening();


  
  // if (!radio.begin()) {
  //   Serial.println(F("radio hardware is not responding!!"));
  //   while (1) {}  // hold in infinite loop
  // }
  // delay(1000);
  Serial.println("alive");
  
  Serial.println("IC2 alive");

  
  #ifdef Ugyro
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
  


  Serial.println("radio Alive");
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

  mainKinematics(testHeightW, 0, 0, aHip,0,0,0);
  delay(200);
  mainKinematics(testHeightW, 0, 0, cHip,0,0,0);
  mainKinematics(testHeightW, 0, 0, bHip,0,0,0);
  delay(200);
  mainKinematics(testHeightW, 0, 0, dHip,0,0,0);

  aLeg.reset();
  bLeg.reset();
  cLeg.reset();
  dLeg.reset();
}

void loop() {
  getData();
  showData();


  
  #ifdef Ugyro  
    bno.getEvent(&event);
    delay(5);
    yPreRot = event.orientation.y;
    zPreRot = event.orientation.z;
   yPID.Compute();
    zPID.Compute();
  #endif
  #ifndef gyro
    yRot = 0;
    zRot = 0;
  #endif

if(payload.eStop != 1){
  pwm.wakeup();
  pwm1.wakeup();
  switch (payload.state) {
    
    case 0:{ //standing
      if(payload.gyro ==1 && payload.PID==1){
        mainKinematics(150,0,0,aHip,0,yRot,zRot);
        mainKinematics(150,0,0,bHip,0,yRot,zRot);
        mainKinematics(150,0,0,cHip,0,yRot,zRot);
        mainKinematics(150,0,0,dHip,0,yRot,zRot);
      }else if(payload.gyro==1 && payload.PID==0){
        mainKinematics(150,0,0,aHip,0,yPreRot,zPreRot);
        mainKinematics(150,0,0,bHip,0,yPreRot,zPreRot);
        mainKinematics(150,0,0,cHip,0,yPreRot,zPreRot);
        mainKinematics(150,0,0,dHip,0,yPreRot,zPreRot);
      }else{
        mainKinematics(150,0,0,aHip,0,0,0);
        mainKinematics(150,0,0,bHip,0,0,0);
        mainKinematics(150,0,0,cHip,0,0,0);
        mainKinematics(150,0,0,dHip,0,0,0);
      }
      break;
    }
    case 1:{ //IK mode
      float xH = payload.j1_x;
      float xLR = payload.j2_x;
      float xFB = payload.j2_y;
      if(payload.gyro == 1 && payload.PID ==1){
        mainKinematics(xH,xFB,xLR,aHip,0,yRot,zRot);
        mainKinematics(xH,xFB,xLR,bHip,0,yRot,zRot);
        mainKinematics(xH,xFB,xLR,cHip,0,yRot,zRot);
        mainKinematics(xH,xFB,xLR,dHip,0,yRot,zRot);
      }else if(payload.gyro == 1 && !payload.PID==1){
        mainKinematics(xH,xFB,xLR,aHip,0,yPreRot,zPreRot);
        mainKinematics(xH,xFB,xLR,bHip,0,yPreRot,zPreRot);
        mainKinematics(xH,xFB,xLR,cHip,0,yPreRot,zPreRot);
        mainKinematics(xH,xFB,xLR,dHip,0,yPreRot,zPreRot);
      }else{
        mainKinematics(xH,xFB,xLR,aHip,0,0,0);
        mainKinematics(xH,xFB,xLR,bHip,0,0,0);
        mainKinematics(xH,xFB,xLR,cHip,0,0,0);
        mainKinematics(xH,xFB,xLR,dHip,0,0,0);
      }
      break;
    }
    case 2:{//FWalk
      if(payload.gyro ==1 && payload.PID== 1){
        WalkF(yRot,zRot, true, testHeightW, testHeightBACKW, testFBW, testLRW,upDistanceW,backDistanceW,LRDistanceW);
      }else if (payload.gyro==1 && !payload.PID== 0){
        WalkF(yPreRot, zPreRot,true, testHeightW, testHeightBACKW, testFBW, testLRW,upDistanceW,backDistanceW,LRDistanceW);
      }else{
        WalkF(0,0,true, testHeightW, testHeightBACKW, testFBW, testLRW, upDistanceW, backDistanceW, LRDistanceW);
      }
      break;
    }
    case 3:{ //
    //void turn(float yRot, float zRot, bool clockwise, float  testHeight, float testHeightBACK, float testFB, float testLR, float upDistance, float backDistance, float LRDistance);Fturn
      if(payload.gyro ==1 && payload.PID== 1){
        turn(yRot,zRot, true, testHeightT,testHeightBACKT,testFBT, testLRT,upDistanceT,backDistanceT,LRDistanceT);
      }else if (payload.gyro ==1 && !payload.PID==0){
        turn(yPreRot, zPreRot, true,testHeightT,testHeightBACKT,testFBT, testLRT,upDistanceT,backDistanceT,LRDistanceT);
      }else{
        turn(0,0,true,testHeightT,testHeightBACKT,testFBT, testLRT,upDistanceT,backDistanceT,LRDistanceT);
      }
      break;
    }
    case 4:{ //user
      float xFB = payload.j2_y;
      float xLR = payload.j2_x;
      float xH = payload.j1_x;
      float timee = payload.j1_y;

      bool direction = true;
      if(xFB < 0 && xLR < 0){
        direction = false;  
      }

      if(payload.gyro == 1 && payload.PID == 1){
        //user controll will be the hardest one
        // float j1_x; forward distance
        // float j1_y; time?
        // float j2_x; xLR
        // float j2_y; xFB
        //bool j1_b; turn 1 way
        //bool j2_b; turn other way
        //turning and moving can NOT mix
        if(!payload.j1_b == 1 && !payload.j2_b == 1){
          WalkF(yRot, zRot, direction,testHeightW,testHeightBACKW, testFBW,testLRW, payload.j1_y,payload.j1_x,payload.j2_x); //need to make WalkF take in values
        }else if (payload.j1_b==1){
          turn(yRot, zRot, true, testHeightT,testHeightBACKT, testFBT,testLRT,payload.j1_x,payload.j1_x,payload.j2_x);
        }else if (payload.j2_b==1){
          turn(yRot, zRot, direction,testHeightT,testHeightBACKT, testFBT,testLRT,payload.j1_y,payload.j1_x,payload.j2_x);
        }
      }else if (payload.gyro == 1 && !payload.PID == 1){
        if(!payload.j1_b == 1 && !payload.j2_b == 1){
          WalkF(yPreRot, zPreRot, direction,testHeightW,testHeightBACKW, testFBW,testLRW, payload.j1_y,payload.j1_x,payload.j2_x); //need to make WalkF take in values
        }else if (payload.j1_b==1){
          turn(yPreRot, zPreRot, true, testHeightT,testHeightBACKT, testFBT,testLRT,payload.j1_x,payload.j1_x,payload.j2_x);
        }else if (payload.j2_b==1){
          turn(yPreRot, zPreRot, direction,testHeightT,testHeightBACKT, testFBT,testLRT,payload.j1_y,payload.j1_x,payload.j2_x);
        }


      }else{
        if(!payload.j1_b == 1 && !payload.j2_b == 1){
          WalkF(0, 0, direction,testHeightW,testHeightBACKW, testFBW,testLRW, payload.j1_y,payload.j1_x,payload.j2_x); //need to make WalkF take in values
        }else if (payload.j1_b==1){
          turn(0, 0, true, testHeightT,testHeightBACKT, testFBT,testLRT,payload.j1_x,payload.j1_x,payload.j2_x);
        }else if (payload.j2_b==1){
          turn(0, 0, direction,testHeightT,testHeightBACKT, testFBT,testLRT,payload.j1_y,payload.j1_x,payload.j2_x);
        }
      }
      break;
    }
      

    default:
      break;
    }
  }else{
    pwm1.sleep();
    pwm.sleep();
  }
}


void getData(){
   if (radio.available()) {
    radio.read( &payload, sizeof(payload) );
    newData = true;
   }else{
    payload.eStop ==true;
   }
}

void showData() {
    if (newData == true) {
        Serial.print("Data received ");
        Serial.println(payload.gyro);
        newData = false;
    }
}