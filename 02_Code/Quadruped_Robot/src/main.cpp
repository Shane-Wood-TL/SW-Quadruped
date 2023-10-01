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


rampLeg aLegR(aHip);
rampLeg bLegR(bHip);
rampLeg cLegR(cHip);
rampLeg dLegR(dHip);

//motor
#define SERVO_FREQ 50
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);



Cords aCords;
Cords bCords;
Cords cCords;
Cords dCords;


//float LlimitV, float HlimitV, bool directionV, float offsetV)
motor aHipM(&pwm, aHip, 45, 135, true, A_HIP_OFFSET);
motor aKneeM(&pwm, aKnee, 0, 180, false, A_KNEE_OFFSET);
motor aAnkleM(&pwm, aAnkle, 0, 180, true, A_ANKLE_OFFSET);

motor bHipM(&pwm, bHip, 45, 135, false, B_HIP_OFFSET);
motor bKneeM(&pwm, bKnee, 0, 180, true, B_KNEE_OFFSET);
motor bAnkleM(&pwm, bAnkle, 0, 180, false, B_ANKLE_OFFSET);

motor cHipM(&pwm1, cHip, 45, 135, false, C_HIP_OFFSET);
motor cKneeM(&pwm1, cKnee, 0, 180, false, C_KNEE_OFFSET);
motor cAnkleM(&pwm1, cAnkle, 0, 180, true, C_ANKLE_OFFSET);

motor dHipM(&pwm1, dHip, 45, 135, true, D_HIP_OFFSET);
motor dKneeM(&pwm1, dKnee, 0, 180, true, D_KNEE_OFFSET);
motor dAnkleM(&pwm1, dAnkle, 0, 180, false, D_ANKLE_OFFSET);

leg Aleg(&aHipM, &aKneeM, &aAnkleM);
leg Bleg(&bHipM, &bKneeM, &bAnkleM);
leg Cleg(&cHipM, &cKneeM, &cAnkleM);
leg Dleg(&dHipM, &dKneeM, &dAnkleM);

kinematics AlegK(&Aleg);
kinematics BlegK(&Bleg);
kinematics ClegK(&Cleg);
kinematics DlegK(&Dleg);



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
Cords basicStand;



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
  // setCycle();
 
  //set legs to stand in some default form
  basicStand.xH = 150;
  AlegK.mainKinematics(basicStand);
  delay(200);
  ClegK.mainKinematics(basicStand);
  BlegK.mainKinematics(basicStand);
  delay(200);
  DlegK.mainKinematics(basicStand);

  //Sets all RAMPS to go to 0 in all joints in all legs
  aLegR.reset();
  bLegR.reset();
  cLegR.reset();
  dLegR.reset();
  movementVariables* walkSetP = &walkSet;
  movementVariables* turnSetP = &turnSet;
  populateStructs(*walkSetP,*turnSetP);
}

void loop() {
  //update radio

  Aleg.setAngles(90,0,0);
  Bleg.setAngles(90,0,0);
  Cleg.setAngles(90,0,0);
  Dleg.setAngles(90,0,0);

  getData();
  
  // //update gyro
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
      aLegR.setCycle(0);
      bLegR.setCycle(3);
      cLegR.setCycle(3);
      dLegR.setCycle(0);
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
          Aleg.setAngles(90,90,0);
          Bleg.setAngles(90,90,0);
          Cleg.setAngles(90,90,0);
          Dleg.setAngles(90,90,0);
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