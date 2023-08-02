#include <Arduino.h>
#include <Ramp.h>
#include <externFunctions.h>

extern ramp aHeight;
extern ramp bHeight;
extern ramp cHeight;
extern ramp dHeight;

extern ramp aFB;
extern ramp bFB;
extern ramp cFB;
extern ramp dFB;

extern ramp aLR;
extern ramp bLR;
extern ramp cLR;
extern ramp dLR;

extern int aCycle;
extern int bCycle;
extern int cCycle;
extern int dCycle;

extern float upDistance;
extern float LRDistance;
extern float timee;
extern float testHeight;
extern float testHeightBACK;

extern int aHip; //
extern int bHip; //
extern int cHip; //
extern int dHip;



void turner(float yRot, float zRot){


  bool direction = true;
  if(aLR.isFinished() && bLR.isFinished() && cLR.isFinished() && dLR.isFinished()){
    Serial.println("yes");
    //void turning(int &Cycle, ramp &LR, ramp &Height, float timee, float LRDistance, float upDistance)
    turning(bCycle, bLR, bHeight, timee, LRDistance, upDistance, direction);
    turning(cCycle, cLR, cHeight, timee, LRDistance, upDistance, direction);
    turning(dCycle, dLR, dHeight, timee, LRDistance, upDistance, direction);
    turning(aCycle, aLR, aHeight, timee, LRDistance, upDistance, direction);
  }

  aFB.update();
  aHeight.update();
  aLR.update();
  if(aCycle == 4 || aCycle == 5 || aCycle == 6){
      mainKinematics(testHeight-aHeight.getValue(), 0, -aLR.getValue(), aHip,0,0,0);
  }else{
      mainKinematics(testHeightBACK-aHeight.getValue(), 0, -aLR.getValue(), aHip,0,yRot,zRot);
  }
  
  cFB.update();
  cHeight.update();
  cLR.update();
  if(cCycle == 4 || cCycle == 5 || cCycle == 6){
      mainKinematics(testHeight-cHeight.getValue(), 0, -cLR.getValue(), cHip,0,0, 0);
  }else{
      mainKinematics(testHeight-cHeight.getValue(), 0, -cLR.getValue(), cHip,0,yRot,zRot);
  }
  
  bFB.update();
  bHeight.update();
  bLR.update();
  if(bCycle == 4 || bCycle == 5 || bCycle == 6){
      mainKinematics(testHeight-bHeight.getValue(), 0, -bLR.getValue(), bHip,0,0,0);
  }else{
      mainKinematics(testHeightBACK-bHeight.getValue(), 0, -bLR.getValue(), bHip,0,yRot,zRot);
  }

  dFB.update();
  dHeight.update();
  dLR.update();
  if(dCycle == 6 || dCycle == 4 || dCycle == 5){
      mainKinematics(testHeight-dHeight.getValue(),0, -dLR.getValue(), dHip,0,0,0);
  }else{
    mainKinematics(testHeight-dHeight.getValue(), 0, -dLR.getValue(), dHip,0,yRot,zRot);
  }
}

void turning(int &Cycle, ramp &LR, ramp &Height, float timee, float LRDistance, float upDistance, bool direction){
float offTime = timee/6.0;
  //Serial.print(Cycle);
  // Serial.println(FB.getTarget());
  if(Cycle == 0 && (LR.isFinished())){
    // Serial.print(Cycle);
    // Serial.println(FB.isFinished());

    LR.go((LRDistance / 4.0), (timee + (3*offTime)));
    if(direction){
        Cycle = 1;
    }else{
        Cycle = 6;
    }
    

  }else if (Cycle == 1 && LR.isFinished())
  {
    // Serial.print(Cycle);
    // Serial.println(FB.isFinished());
    LR.go(((2*LRDistance)/4.0), timee);
    if(direction){
      Cycle = 2;
    }else{
        Cycle = 0;
    }

  }else if (Cycle == 2 && LR.isFinished())
  {
    // Serial.print(Cycle);
    // Serial.println(FB.isFinished());
    LR.go((3*LRDistance/4.0),timee);
    Height.go(0, timee);
    if(direction){
        Cycle = 3;
    }else{
        Cycle = 1;
    }

  }else if (Cycle == 3 && LR.isFinished())
  {
    // Serial.print(Cycle);
    // Serial.println(FB.isFinished());
    LR.go(LRDistance, timee);
    Height.go(0,timee);
    
    if(direction){
    Cycle = 4;
    }else{
        Cycle = 2;
    }

  }
  else if(Cycle == 4 && LR.isFinished())
  {
    // Serial.print(Cycle);
    // Serial.println(FB.isFinished());
    Height.go(upDistance,offTime);
    LR.go(((LRDistance) /4.0), offTime);
    if(direction){
    Cycle = 5;
    }else{
        Cycle = 3;
    }
  }  else if(Cycle == 5 && LR.isFinished())
  {
    // Serial.print(Cycle);
    // Serial.println(FB.isFinished());
    Height.go(upDistance,offTime);
    LR.go((LRDistance /4.0), offTime);
    if(direction){
        Cycle = 6;
    }else{
        Cycle = 4;
    }
    
  }  else if(Cycle == 6 && LR.isFinished())
  {
    // Serial.print(Cycle);
    // Serial.println(FB.isFinished());
    Height.go(0,offTime);
    LR.go(0, offTime);
    if (direction){
    Cycle = 0;
    }else{
        Cycle = 5;
    }
  }


}