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
#include <externFunctions.h>


#define Ugyro

//SCK 18
//MOSI 8
//MISO 10
//CE 16
//CSN 9






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

PayloadStruct payload;

const byte thisSlaveAddress[5] = {'R','x','A','A','A'};
RF24 radio(16,9);

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
  Serial.println("radio Alive");

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
  #ifndef Ugyro
    yRot = 0;
    zRot = 0;
  #endif

if(payload.eStop != 1){
  wakeup_9();
  if(oldState !=payload.state){
    aLeg.setCycle(0);
    bLeg.setCycle(3);
    cLeg.setCycle(3);
    dLeg.setCycle(0);
  }
  switch (payload.state) {
    case 0:{ //standing
      standing_0();
      break;
    }
    case 1:{ //IK mode
      IK_1();
      break;
    }
    case 2:{//FWalk
      FWalk_2();
      break;
    }
    case 3:{ //
      FTurn_3();
      break;
    }
    case 4:{ //user
      User_4();
      break;
    } 
    case 5:{ //used to install new motors
      all_90s();
    }
    default:
      break;
    }
  }else{
    Default_9(); //turns off motors
  }
  oldState = payload.state;
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