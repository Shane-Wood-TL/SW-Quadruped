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

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <TelnetStream.h>

//#define OTA
#define CONTROLLER
#define CONTROLLERA
//#define TELNET

//------------------------------------------------------------------------------------------------

// pload_protocol = espota
// upload_port = QuadrupedRobot.local

// const char* ssid = "SSID";
// const char* password = "password";



//------------------------------------------------------------------------------------------------
//angle goal values
double xAngle = 0;
double yAngle = 0;
double zAngle = 0;

double xAngleV = 0;
double yAngleV = 0;
double zAngleV = 0;


//------------------------------------------------------------------------------------------------
//angle variables
double yPreRot= 0, zPreRot = 0;
double yRot= 0, zRot = 0;


//------------------------------------------------------------------------------------------------
//PID setup for gyro
double Kp= .25, Ki=.01, Kd =0;
double angleGoal = 0;


//------------------------------------------------------------------------------------------------
//motor driver setup
#define SERVO_FREQ 50
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x60);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x7C);


//------------------------------------------------------------------------------------------------
//location variables
Cords aCords;
Cords bCords;
Cords cCords;
Cords dCords;


//------------------------------------------------------------------------------------------------
// Motor, Leg, kinematics, interpolation set up
//float LlimitV, float HlimitV, bool directionV, float offsetV)
motor aHipM(&pwm, aHip, 45, 135, true, A_HIP_OFFSET);
motor aKneeM(&pwm, aKnee, 0, 180, false, A_KNEE_OFFSET);
motor aAnkleM(&pwm, aAnkle, 0, 180, false, A_ANKLE_OFFSET);

motor bHipM(&pwm, bHip, 45, 135, false, B_HIP_OFFSET);
motor bKneeM(&pwm, bKnee, 0, 180, true, B_KNEE_OFFSET);
motor bAnkleM(&pwm, bAnkle, 0, 180, true, B_ANKLE_OFFSET);

motor cHipM(&pwm1, cHip, 45, 135, false, C_HIP_OFFSET);
motor cKneeM(&pwm1, cKnee, 0, 180, false, C_KNEE_OFFSET);
motor cAnkleM(&pwm1, cAnkle, 0, 180, false, C_ANKLE_OFFSET);

motor dHipM(&pwm1, dHip, 45, 135, true, D_HIP_OFFSET);
motor dKneeM(&pwm1, dKnee, 0, 180, true, D_KNEE_OFFSET);
motor dAnkleM(&pwm1, dAnkle, 0, 180, true, D_ANKLE_OFFSET);


//------------------------------------------------------------------------------------------------
//each leg object
leg Aleg(&aHipM, &aKneeM, &aAnkleM, "A");
leg Bleg(&bHipM, &bKneeM, &bAnkleM, "B");
leg Cleg(&cHipM, &cKneeM, &cAnkleM, "C");
leg Dleg(&dHipM, &dKneeM, &dAnkleM, "D");


//------------------------------------------------------------------------------------------------
//kinematic objects
kinematics AlegK(&Aleg);
kinematics BlegK(&Bleg);
kinematics ClegK(&Cleg);
kinematics DlegK(&Dleg);


//------------------------------------------------------------------------------------------------
//Ramp(interpolation) objects
rampLeg aLegR(aHip);
rampLeg bLegR(bHip);
rampLeg cLegR(cHip);
rampLeg dLegR(dHip);


//------------------------------------------------------------------------------------------------
//gyro
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
sensors_event_t event;


//------------------------------------------------------------------------------------------------
//PID creation
PID yPID(&yPreRot, &yRot, &angleGoal, Kp, Ki, Kd, DIRECT);
PID zPID(&zPreRot, &zRot, &angleGoal, Kp, Ki, Kd, DIRECT);


//------------------------------------------------------------------------------------------------
//radio
PayloadStruct payload;
//RF24 radio(ce, csn)
RF24 radio(14,13);
bool newData = false;
#ifdef CONTROLLER
int oldState = 0;
const byte thisSlaveAddress[5] = {'R','x','A','A','A'};
#endif

//CE = 12, 
//CSN = 18
//sck = 13
//MISO = 17
//MOSI = 8
//------------------------------------------------------------------------------------------------
//more (different) location variables
movementVariables walkSet;
movementVariables turnSet;
Cords basicStand;



void setup() {
  //start busses
  //start Serial
  Serial.begin(9600);
  Serial.println("Serial Active");


  //------------------------------------------------------------------------------------------------
  //Start I2C
  Wire.begin(04,05); //SDA, SCL
  Serial.println("IC2 Active");

  //------------------------------------------------------------------------------------------------
  //start SPI bus for NRF2401
  #ifdef CONTROLLER
    //SPI.begin(SCK, MISO, MOSI);
    SPI.begin(12, 10, 11);
    //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  #endif

  //------------------------------------------------------------------------------------------------
  //enable OTA
  #ifdef OTA
  ArduinoOTA.setHostname("QuadrupedRobot");
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  
  // Print the ESP32's local IP address
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Setup OTA
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {
      type = "filesystem";
    }
    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  #endif


  //------------------------------------------------------------------------------------------------
  //Start Telent, (serial over wifi)
  #ifdef TELNET
  TelnetStream.begin();
  #endif


  //------------------------------------------------------------------------------------------------
  //enable the controller
  #ifdef CONTROLLER
  //start radio
    radio.begin();
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.openReadingPipe(1, thisSlaveAddress);
    radio.startListening();
    Serial.println("radio Active");
  #endif


  // //------------------------------------------------------------------------------------------------
  // //Set up PCA9685s / motor drivers
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);  
  pwm1.begin();
  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates


  // //------------------------------------------------------------------------------------------------
  // start gyro
  radio.printPrettyDetails();
  bno.begin();
  bno.setExtCrystalUse(true);
  Serial.println("gyro Started");

  if(!bno.begin()){
    Serial.print("Gyro Error");
    while(1);
  }

  delay(1000); 
  Serial.println("Gyro Active");


  // //------------------------------------------------------------------------------------------------
  // //limit PID values to keep motors in safe range
  yPID.SetOutputLimits(-20.0,20.0);
  zPID.SetOutputLimits(-20.0,20.0);

  //set the pid mode
  yPID.SetMode(AUTOMATIC);
  zPID.SetMode(AUTOMATIC);
  Serial.println("PID Active");


  // //------------------------------------------------------------------------------------------------
  //where in the walk cycle the robot is at. Way to share state across all movements
  setCycle();

  //set legs to stand in some default form
  basicStand.xH = 100;
  basicStand.xLR = 0;
  basicStand.xFB = -20;
  standing_0();

  // Sets all RAMPS to go to 0 in all joints in all legs
  resetAll();

  movementVariables* walkSetP = &walkSet;
  movementVariables* turnSetP = &turnSet;
  populateStructs(*walkSetP,*turnSetP);
}

void loop() {  
  #ifdef OTA
  ArduinoOTA.handle();
  #endif
  
  //standing_0();
  
  //bKneeM.setDegree(0);
  //FWalk_2(yAngleV, zAngleV);
  //update radio

  getData();
  //payload.state == 0;
  //------------------------------------------------------------------------------------------------
  // //update gyro
  bno.getEvent(&event);
  delay(5);
  if(payload.gyro == 1){
    yPreRot = event.orientation.y;
    zPreRot = event.orientation.z;
    if(payload.PID == 1){
      yPID.Compute();
      zPID.Compute();
      //yAngleV = yRot;
      //zAngleV = zRot;
    }else{
      yAngleV =  event.orientation.y;
      zAngleV =  event.orientation.z;
    }
  }else{
    yAngleV = 0;
    zAngleV = 0;
  }

  
  //------------------------------------------------------------------------------------------------
  #ifdef CONTROLLERA
    //switch modes and saftey mode
    if(payload.eStop != 1){
      wakeup_9();
      if(oldState !=payload.state){
          // aLegR.setCycle(0);
          // bLegR.setCycle(3);
          // cLegR.setCycle(3);
          // dLegR.setCycle(0);
      }
      switch (payload.state) {
        case 0:{ //standing
            standing_0();
          break;
        }
        case 1:{ //IK mode
            IK_1(0,0,0);
          break;
        }
        case 2:{//FWalk
            FWalk_2(xAngleV,yAngleV);
          break;
        }
        case 3:{ //
            FTurn_3(xAngleV,yAngleV);
          break;
        }
        case 4:{ //user
            User_4(xAngleV,yAngleV);
          break;
        } 
        case 5:{ //used to install new motors
              Aleg.setAngles(90,180,0);
              Bleg.setAngles(90,180,0);
              Cleg.setAngles(90,180,0);
              Dleg.setAngles(90,180,0);
            break;
        }
        default:
          break;
        }
    }else{
        Default_9(); //turns off motors
    }
    oldState = payload.state;
  #endif
}

//gets data from radio, checks if data was recieved
void getData(){
   if (radio.available()) {
    radio.read(&payload, sizeof(payload));
    newData = true;
  //  }else{
  //   payload.eStop ==true; 
  }

}