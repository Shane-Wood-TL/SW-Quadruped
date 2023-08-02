#include <Arduino.h>
#include <Ramp.h>
#include <externFunctions.h>

extern float timee;

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

extern float backDistance;
extern float upDistance;

extern float testHeight;
extern float testLR;

extern float testHeightBACK;

//motor definitions
extern int aHip; //
extern int bHip; //
extern int cHip; //
extern int dHip;

void WalkF(float yRot, float zRot){


  

  if(aFB.isFinished() && bFB.isFinished() && cFB.isFinished() && dFB.isFinished()){
    walk(bCycle, bFB, bHeight, timee, backDistance, upDistance, true);
    walk(cCycle, cFB, cHeight, timee, backDistance, upDistance, true);
    walk(dCycle, dFB, dHeight, timee, backDistance, upDistance, true);
    walk(aCycle, aFB, aHeight, timee, backDistance, upDistance, true);
  }

  aFB.update();
  aHeight.update();
  aLR.update();
  if(aCycle == 4 || aCycle == 5 || aCycle == 6){
    mainKinematics(testHeight-aHeight.getValue(), -aFB.getValue(), testLR, aHip,0,0,0);
  }else{
    mainKinematics(testHeight-aHeight.getValue(), -aFB.getValue(), testLR, aHip,0,yRot,zRot);
  }
  

  cFB.update();
  cHeight.update();
  cLR.update();
  if(cCycle == 4 || cCycle == 5 || cCycle == 6){
    mainKinematics(testHeightBACK-cHeight.getValue(), -cFB.getValue(), testLR, cHip,0,0, 0);
  }else{
    mainKinematics(testHeightBACK-cHeight.getValue(), -cFB.getValue(), testLR, cHip,0,yRot,zRot);
  }

  

  bFB.update();
  bHeight.update();
  bLR.update();
  if(bCycle == 4 || bCycle == 5 || bCycle == 6){
    mainKinematics(testHeight-bHeight.getValue(), -bFB.getValue(), testLR, bHip,0,0,0);
  }else{
    mainKinematics(testHeight-bHeight.getValue(), -bFB.getValue(), testLR, bHip,0,yRot,zRot);
  }


  dFB.update();
  dHeight.update();
  dLR.update();
  if(dCycle == 6 || dCycle == 4 || dCycle == 5){
    mainKinematics(testHeightBACK-dHeight.getValue(), -dFB.getValue(), testLR, dHip,0,0,0);
  }else{
   mainKinematics(testHeightBACK-dHeight.getValue(), -dFB.getValue(), testLR, dHip,0,yRot,zRot);
  }
}

void walk(int &Cycle, ramp &FB, ramp &Height, float timee, float backDistance, float upDistance, bool d = true){
  float offTime = timee/6.0;
  //Serial.print(Cycle);
  // Serial.println(FB.getTarget());
  if(Cycle == 0 && (FB.isFinished())){
    // Serial.print(Cycle);
    // Serial.println(FB.isFinished());

    FB.go((backDistance / 4.0), (timee + (3*offTime)));
    if (d){
    Cycle = 1;
    }else{
      Cycle = 6;
    }

  }else if (Cycle == 1 && FB.isFinished())
  {
    // Serial.print(Cycle);
    // Serial.println(FB.isFinished());
    FB.go(((2*backDistance)/4.0), timee);
    if(d){
    Cycle = 2;
    }else{
      Cycle = 0;
    }

  }else if (Cycle == 2 && FB.isFinished())
  {
    // Serial.print(Cycle);
    // Serial.println(FB.isFinished());
    FB.go((3*backDistance/4.0),timee);
    Height.go(0, timee);
    if(d){
    Cycle = 3;
    }else{
      Cycle = 1;
    }

  }else if (Cycle == 3 && FB.isFinished())
  {
    // Serial.print(Cycle);
    // Serial.println(FB.isFinished());
    FB.go(backDistance, timee);
    Height.go(0,timee);
    if(d){
    Cycle = 4;
    }else{
      Cycle = 2;
    }

  }
  else if(Cycle == 4 && FB.isFinished())
  {
    // Serial.print(Cycle);
    // Serial.println(FB.isFinished());
    Height.go(upDistance,offTime);
    FB.go(((backDistance) /4.0), offTime);
    if(d){
    Cycle = 5;
    }else{
      Cycle = 3;
    }

  }else if(Cycle == 5 && FB.isFinished())
  {
    // Serial.print(Cycle);
    // Serial.println(FB.isFinished());
    Height.go(upDistance,offTime);
    FB.go((backDistance /4.0), offTime);
    if (d){
    Cycle = 6;
    }else{
      Cycle = 4;
    }
  }  else if(Cycle == 6 && FB.isFinished())
  {
    // Serial.print(Cycle);
    // Serial.println(FB.isFinished());
    Height.go(0,offTime);
    FB.go(0, offTime);
    if (d){
    Cycle = 0;
    }else{
      Cycle = 5;
    }
  }

}


void resetAll(){
  
  aHeight.go(0,0);
  aFB.go(0,0);

  bHeight.go(0,0);
  bFB.go(0,0);

  cHeight.go(0,0);
  cFB.go(0,0);

  dHeight.go(0,0);
  dFB.go(0,0);

  aLR.go(0,0);
  bLR.go(0,0);
  cLR.go(0,0);
  dLR.go(0,0);

}




void WalkLR(float yRot, float zRot, bool d){
  if(aLR.isFinished() && bLR.isFinished() && cLR.isFinished() && dLR.isFinished()){
    Serial.println("yes");
    //void turning(int &Cycle, ramp &LR, ramp &Height, float timee, float LRDistance, float upDistance)
    walk(bCycle, bLR, bHeight, timee, 20, upDistance,d);
    walk(cCycle, cLR, cHeight, timee, 20, upDistance,d);
    walk(dCycle, dLR, dHeight, timee, 20, upDistance,d);
    walk(aCycle, aLR, aHeight, timee, 20, upDistance,d);
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
      mainKinematics(testHeight-bHeight.getValue(), 0, bLR.getValue(), bHip,0,0,0);
  }else{
      mainKinematics(testHeightBACK-bHeight.getValue(), 0, bLR.getValue(), bHip,0,yRot,zRot);
  }

  dFB.update();
  dHeight.update();
  dLR.update();
  if(dCycle == 6 || dCycle == 4 || dCycle == 5){
      mainKinematics(testHeight-dHeight.getValue(),0, dLR.getValue(), dHip,0,0,0);
  }else{
    mainKinematics(testHeight-dHeight.getValue(), 0, dLR.getValue(), dHip,0,yRot,zRot);
  }
}