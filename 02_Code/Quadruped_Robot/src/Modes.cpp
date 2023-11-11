#include <Arduino.h>
#include <externFunctions.h>
#include <Adafruit_PWMServoDriver.h>

//------------------------------------------------------------------------------------------------
extern Cords aCords;
extern Cords bCords;
extern Cords cCords;
extern Cords dCords;


extern PayloadStruct payload;
extern Adafruit_PWMServoDriver pwm;
extern Adafruit_PWMServoDriver pwm1;

extern kinematics AlegK;
extern kinematics BlegK;
extern kinematics ClegK;
extern kinematics DlegK;


extern rampLeg aLegR;
extern rampLeg bLegR;
extern rampLeg cLegR;
extern rampLeg dLegR;

extern movementVariables walkSet;
extern movementVariables turnSet;
extern Cords basicStand;


//------------------------------------------------------------------------------------------------
void standing_0(){
  AlegK.mainKinematics(basicStand);
  BlegK.mainKinematics(basicStand);
  ClegK.mainKinematics(basicStand);
  DlegK.mainKinematics(basicStand);
}


//------------------------------------------------------------------------------------------------
void IK_1(float xAngleV, float yAngleV, float zAngleV){
  float xH = payload.j1_x;
  float xLR = payload.j2_x;
  float xFB = payload.j2_y;
  Cords pos;
  pos.updateCords(xH,xFB,xLR,xAngleV,yAngleV,zAngleV);
  AlegK.mainKinematics(pos);
  BlegK.mainKinematics(pos);
  ClegK.mainKinematics(pos);
  DlegK.mainKinematics(pos);
}


//------------------------------------------------------------------------------------------------
void FWalk_2(float yAngleV, float zAngleV){
  Serial.println(yAngleV);
  WalkF(yAngleV,zAngleV, true, testHeightW, testHeightBACKW, testFBW, testLRW,upDistanceW,backDistanceW,LRDistanceW);
}


//------------------------------------------------------------------------------------------------
void FTurn_3(float yAngleV, float zAngleV){
  //void turn(float yRot, float zRot, bool clockwise, float  testHeight, float testHeightBACK, float testFB, float testLR, float upDistance, float backDistance, float LRDistance);Fturn
  turn(yAngleV,zAngleV, true, testHeightT,testHeightBACKT,testFBT, testLRT,upDistanceT,backDistanceT,LRDistanceT);
}


//------------------------------------------------------------------------------------------------
void User_4(float yAngleV, float zAngleV)
{
  float xFB = payload.j2_y;
  float xLR = payload.j2_x;
  float xH = payload.j1_y;

  bool direction = true;
  if (xFB < 0 || xLR < 0){
    direction = false;
  }

  if (!payload.j1_b == 1 && !payload.j2_b == 1){
    // WalkF( yRot,zRot, direction,testHeight, testHeightBACK,  testFB, testLR,  upDistance,   backDistance, LRDistance)
    WalkF(yAngleV, zAngleV, direction, testHeightW, testHeightBACKW, testFBW, testLRW, xH, xFB, xLR); // need to make WalkF take in values
  }
  else if (payload.j1_b == 1){
    turn(yAngleV, zAngleV, true, testHeightT, testHeightBACKT, testFBT, testLRT, xH, xFB, xLR);
  }
  else if (payload.j2_b == 1){
    turn(yAngleV, zAngleV, direction, testHeightT, testHeightBACKT, testFBT, testLRT, xH, xFB, xLR);
  }
}


//------------------------------------------------------------------------------------------------
void Default_9(){
  pwm.sleep();
  pwm1.sleep();
}


//------------------------------------------------------------------------------------------------
void wakeup_9(){
  pwm.wakeup();
  pwm1.wakeup();
}


//------------------------------------------------------------------------------------------------
void setCycle(){
  aLegR.setCycle(0);
  bLegR.setCycle(3);
  cLegR.setCycle(3);
  dLegR.setCycle(0);
}
