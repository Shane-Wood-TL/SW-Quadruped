#ifndef __function_declarations__
#define __function_declarations__


#include "../include/motion_system/motor.h"
#include "../include/motion_system/singleCycleClass.h"
#include "../include/motion_system/kinematicsClass.h"
#include "../include/motion_system/legClass.h"
#include "../include/motion_system/rampLegClass.h"
#include "../include/motion_system/movementCyclesClass.h"
#include "../include/motion_system/cycleControlClass.h"
#include "../include/hardware_setup/motor_offsets.h"









struct PayloadStruct {
  uint8_t eStop; //sw2
  uint8_t state;
  uint8_t gyro;
  uint8_t PID;
  int16_t j1_x;
  int16_t j1_y;
  int16_t j2_x;
  int16_t j2_y;
  uint8_t bt0_;
  uint8_t bt1_;
};



//Movement
void walk(rampLeg &Leg, float timee, float backDistance, float upDistance, float LRDistance, bool d);
void WalkF(float yRot, float zRot, bool direction, float  testHeight, float testHeightBACK, float testFB, float testLR, float upDistance, float backDistance, float LRDistance);
void turn(float yRot, float zRot, bool clockwise, float  testHeight, float testHeightBACK, float testFB, float testLR, float upDistance, float backDistance, float LRDistance);


//------------------------------------------------------------------------------------------------
//Modes
void standing_0();
void IK_1(float xAngleV, float yAngleV, float zAngleV);
void FWalk_2(float yAngleV, float zAngleV);
void FTurn_3(float yAngleV, float zAngleV);
void User_4(float yAngleV, float zAngleV);

void Default_9();
void wakeup_9();

void resetAll();
void updateAll();
bool allDone();




void standing_0();
//------------------------------------------------------------------------------------------------
void IK_1(float xAngleV, float yAngleV, float zAngleV);


//------------------------------------------------------------------------------------------------
void FWalk_2(float yAngleV, float zAngleV);


//------------------------------------------------------------------------------------------------
void FTurn_3(float yAngleV, float zAngleV);

//------------------------------------------------------------------------------------------------
void User_4(float yAngleV, float zAngleV);

//------------------------------------------------------------------------------------------------
void Default_9();


//------------------------------------------------------------------------------------------------
void wakeup_9();

//------------------------------------------------------------------------------------------------
void setCycle();
#endif