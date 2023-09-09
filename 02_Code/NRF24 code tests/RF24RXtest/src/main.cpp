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
// lib_deps = 
// 	nrf24/RF24@^1.4.7
// 	adafruit/Adafruit PWM Servo Driver Library@^2.4.1
// 	adafruit/Adafruit BNO055@^1.6.1
// 	siteswapjuggler/Ramp@^0.6.1
// 	br3ttb/PID@^1.2.1
// 18 Mar 2018 - simple program to verify connection between Arduino
//      and nRF24L01+
//  This program does NOT attempt any communication with another nRF24
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


    //   delay(1000); 
    // Serial.println("gyroalive");
    // yPID.SetOutputLimits(-45.0,45.0);
    // zPID.SetOutputLimits(-45.0,45.0);
    // yPID.SetMode(AUTOMATIC);
    // zPID.SetMode(AUTOMATIC);
    // Serial.println("PID alive");
    // Serial.println("radio Alive");
    // //Set up PCA9685
    // pwm.begin();
    // pwm.setOscillatorFrequency(27000000);
    // pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
    // delay(10);  

    // pwm1.begin();
    // pwm1.setOscillatorFrequency(27000000);
    // pwm1.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
    // Serial.println("CheckConnection Starting");
    // Serial.println();
    // Serial.println("FIRST WITH THE DEFAULT ADDRESSES after power on");
    // Serial.println("  Note that RF24 does NOT reset when Arduino resets - only when power is removed");
    // Serial.println("  If the numbers are mostly 0x00 or 0xff it means that the Arduino is not");
    // Serial.println("     communicating with the nRF24");
    // Serial.println();
    // radio.begin();
    // radio.printDetails();
    // Serial.println();
    // Serial.println();
    // Serial.println("AND NOW WITH ADDRESS AAAxR  0x41 41 41 78 52   ON P1");
    // Serial.println(" and 250KBPS data rate");
    // Serial.println();
    // radio.openReadingPipe(1, thisSlaveAddress);
    // radio.setDataRate( RF24_250KBPS );
    // radio.printDetails();
    // Serial.println();
    // Serial.println();
}


void loop() {
    getData();
    showData();
}

void getData() {
    if ( radio.available() ) {
        radio.read( &payload, sizeof(payload) );
        newData = true;
    }
}

void showData() {
    if (newData == true) {
        Serial.print("Data received ");
        Serial.println(payload.gyro);
        newData = false;
    }
}