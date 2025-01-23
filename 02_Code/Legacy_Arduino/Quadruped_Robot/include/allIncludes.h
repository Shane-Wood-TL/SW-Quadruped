//Arduino Base Libraries
#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <Wire.h>

//Arduino Libraries
#include <nRF24L01.h>
#include <RF24.h>
#include <Ramp.h>
#include <Adafruit_PWMServoDriver.h>
#include <utility/imumaths.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <PID_v1.h> 

//Kinematics
#include <Kinematics/motorClass.h>
#include <Kinematics/legClass.h>
#include <Kinematics/kinematicsClass.h>

//Motion
#include <Motion/cordsClass.h>
#include <Motion/rampLegClass.h>
#include <Motion/singleCycleClass.h>
#include <Motion/movementCyclesClass.h>
#include <Motion/cycleControlClass.h>

//Wireless
#include <Wireless/payloadStruct.h>

//Function Calls
#include <externFunctions.h>

//Constants
#include <motorOffsets.h>
#include <robotConstants.h>





