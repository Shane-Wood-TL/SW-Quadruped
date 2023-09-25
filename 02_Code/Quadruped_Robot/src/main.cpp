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

//angle variables
double yPreRot= 0, zPreRot = 0;
double yRot= 0, zRot = 0;

//PID setup for gyro
double Kp= .25, Ki=.01, Kd =0;
double angleGoal = 0;


// DO NOT CHANGE ANY VARIABLE BELOW THIS LINE (unless you know what you are doing)
//motor definitions
// old way to desinate each leg, legacy, but the kinematic model depends on these values.
//leg
int aHip = 0; //
int bHip = 3; //
int cHip = 6; //
int dHip = 9; //

rampLeg aLeg(aHip);
rampLeg bLeg(bHip);
rampLeg cLeg(cHip);
rampLeg dLeg(dHip);

//motor
#define SERVO_FREQ 50
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);

//gyro
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
sensors_event_t event;

PID yPID(&yPreRot, &yRot, &angleGoal, Kp, Ki, Kd, DIRECT);
PID zPID(&zPreRot, &zRot, &angleGoal, Kp, Ki, Kd, DIRECT);

//radio
int oldState = 0;
PayloadStruct payload;
const byte thisSlaveAddress[5] = {'R','x','A','A','A'};
RF24 radio(16,9);
bool newData = false;

movementVariables walkSet;
movementVariables turnSet;


void setup() {
  //start busses
  Serial.begin(9600);
  Serial.println("Serial Active");
  
  Wire.begin(17,15); //SDA, SCL
  Serial.println("IC2 Active");

  //SPI.begin(SCK, MISO, MOSI);
  SPI.begin(18, 8, 10);

  //start radio
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.openReadingPipe(1, thisSlaveAddress);
  radio.startListening();
  Serial.println("radio Active");

  //start gyro
  bno.begin();
  Serial.println("gyro Started");
  if(!bno.begin()){
    Serial.print("Gyro Error");
    while(1);
  }
  //bno.setExtCrystalUse(true);
  delay(1000); 
  Serial.println("Gyro Active");

  //limit PID values to keep motors in safe range
  yPID.SetOutputLimits(-45.0,45.0);
  zPID.SetOutputLimits(-45.0,45.0);

  //set the pid mode
  yPID.SetMode(AUTOMATIC);
  zPID.SetMode(AUTOMATIC);
  Serial.println("PID Active");

  
  //Set up PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);  
  pwm1.begin();
  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates


  //where in the walk cycle the robot is at. Way to share state across all movements
  setCycle();
  
  //set legs to stand in some default form
  mainKinematics(testHeightW, 0, 0, aHip,0,0,0);
  delay(200); //prevents OCP
  mainKinematics(testHeightW, 0, 0, cHip,0,0,0);
  mainKinematics(testHeightW, 0, 0, bHip,0,0,0);
  delay(200);
  mainKinematics(testHeightW, 0, 0, dHip,0,0,0);

  //Sets all RAMPS to go to 0 in all joints in all legs
  aLeg.reset();
  bLeg.reset();
  cLeg.reset();
  dLeg.reset();
  movementVariables* walkSetP = &walkSet;
  movementVariables* turnSetP = &turnSet;
  populateStructs(*walkSetP,*turnSetP);
}

void loop() {
  //update radio
  getData();
  
  //update gyro
  if(payload.gyro == 1){
    bno.getEvent(&event);
    delay(5);
    yPreRot = event.orientation.y;
    zPreRot = event.orientation.z;
    if(payload.PID == 1){
      yPID.Compute();
      zPID.Compute();
    }
  }else{
    yRot = 0;
    zRot = 0;
  }

  //switch modes and saftey mode
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


//gets data from radio, checks if data was recieved
void getData(){
   if (radio.available()) {
    radio.read( &payload, sizeof(payload) );
    newData = true;
  //  }else{
  //   payload.eStop ==true; 
  }

}