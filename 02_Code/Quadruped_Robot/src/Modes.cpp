#include <Arduino.h>
#include <externFunctions.h>
#include <Adafruit_PWMServoDriver.h>

extern int aHip;
extern int bHip;
extern int cHip;
extern int dHip;

extern float yRot;
extern float zRot;
extern float yPreRot;
extern float zPreRot;
extern const float testHeightW; 
extern const float testHeightBACKW;
extern const float testFBW;
extern const float testLRW;
extern const float upDistanceW;
extern const float backDistanceW;
extern const float LRDistanceW;

extern const float testHeightT;
extern const float testHeightBACKT;
extern const float testFBT;
extern const float  testLRT;
extern const float upDistanceT;
extern const float backDistanceT;
extern const float LRDistanceT;


extern PayloadStruct payload;
extern Adafruit_PWMServoDriver pwm;
extern Adafruit_PWMServoDriver pwm1;


extern rampLeg aLeg;
extern rampLeg bLeg;
extern rampLeg cLeg;
extern rampLeg dLeg;


void standing_0(){
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
}

void IK_1(){
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
}

void FWalk_2(){
    if(payload.gyro ==1 && payload.PID== 1){
        WalkF(yRot,zRot, true, testHeightW, testHeightBACKW, testFBW, testLRW,upDistanceW,backDistanceW,LRDistanceW);
    }else if (payload.gyro==1 && !payload.PID== 0){
        WalkF(yPreRot, zPreRot,true, testHeightW, testHeightBACKW, testFBW, testLRW,upDistanceW,backDistanceW,LRDistanceW);
    }else{
        WalkF(0,0,true, testHeightW, testHeightBACKW, testFBW, testLRW, upDistanceW, backDistanceW, LRDistanceW);
    }
}

void FTurn_3(){
//void turn(float yRot, float zRot, bool clockwise, float  testHeight, float testHeightBACK, float testFB, float testLR, float upDistance, float backDistance, float LRDistance);Fturn
      if(payload.gyro ==1 && payload.PID== 1){
        turn(yRot,zRot, true, testHeightT,testHeightBACKT,testFBT, testLRT,upDistanceT,backDistanceT,LRDistanceT);
      }else if (payload.gyro ==1 && !payload.PID==0){
        turn(yPreRot, zPreRot, true,testHeightT,testHeightBACKT,testFBT, testLRT,upDistanceT,backDistanceT,LRDistanceT);
      }else{
        turn(0,0,true,testHeightT,testHeightBACKT,testFBT, testLRT,upDistanceT,backDistanceT,LRDistanceT);
      }
}

void User_4(){
    
      float xFB = payload.j2_y;
      float xLR = payload.j2_x;
      float xH = payload.j1_y;

      bool direction = true;
      if(xFB < 0 && xLR < 0){
        direction = false;  
      }
      xFB = abs(xFB);
      xLR = abs(xLR);

      if(payload.gyro == 1 && payload.PID == 1){
        if(!payload.j1_b == 1 && !payload.j2_b == 1){
        //WalkF( yRot,zRot, direction,testHeight, testHeightBACK,  testFB, testLR,  upDistance,   backDistance, LRDistance)
          WalkF(yRot, zRot, direction,testHeightW,testHeightBACKW, testFBW,testLRW, xH, xFB, xLR); //need to make WalkF take in values
        }else if (payload.j1_b==1){
          turn(yRot, zRot, true, testHeightT,testHeightBACKT, testFBT,testLRT, xH, xFB, xLR);
        }else if (payload.j2_b==1){
          turn(yRot, zRot, direction,testHeightT,testHeightBACKT, testFBT,testLRT, xH, xFB, xLR);
        }
      }else if (payload.gyro == 1 && !payload.PID == 1){
        if(!payload.j1_b == 1 && !payload.j2_b == 1){
          WalkF(yPreRot, zPreRot, direction,testHeightW,testHeightBACKW, testFBW,testLRW, xH, xFB, xLR); //need to make WalkF take in values
        }else if (payload.j1_b==1){
          turn(yPreRot, zPreRot, true, testHeightT,testHeightBACKT, testFBT,testLRT, xH, xFB, xLR);
        }else if (payload.j2_b==1){
          turn(yPreRot, zPreRot, direction,testHeightT,testHeightBACKT, testFBT,testLRT, xH, xFB, xLR);
        }


      }else{
        if(!payload.j1_b == 1 && !payload.j2_b == 1){
          WalkF(0, 0, direction,testHeightW,testHeightBACKW, testFBW,testLRW,  xH, xFB, xLR); //need to make WalkF take in values
        }else if (payload.j1_b==1){
          turn(0, 0, true, testHeightT,testHeightBACKT, testFBT,testLRT, xH, xFB, xLR);
        }else if (payload.j2_b==1){
          turn(0, 0, direction,testHeightT,testHeightBACKT, testFBT,testLRT, xH, xFB, xLR);
        }
      }
}

void Default_9(){
    pwm.sleep();
    pwm1.sleep();
}

void wakeup_9(){
pwm.wakeup();
  pwm1.wakeup();
}

void setCycle(){
  aLeg.setCycle(0);
  bLeg.setCycle(3);
  cLeg.setCycle(3);
  dLeg.setCycle(0);
}
