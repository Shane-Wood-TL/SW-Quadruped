#include <Arduino.h>
#include <Ramp.h>
#include <externFunctions.h>

extern float timee;

extern float backDistance;
extern float upDistance;
extern float LRDistance;

extern float testHeight;
extern float testHeightBACK;

extern rampLeg aLeg;
extern rampLeg bLeg;
extern rampLeg cLeg;
extern rampLeg dLeg;


void WalkF(float yRot, float zRot){

  if(aLeg.allDone() && bLeg.allDone() && cLeg.allDone() && dLeg.allDone()){
    walk(bLeg, timee, backDistance, upDistance, LRDistance, true);
    walk(cLeg, timee, backDistance, upDistance, LRDistance, true);
    walk(dLeg, timee, backDistance, upDistance, LRDistance, true);
    walk(aLeg, timee, backDistance, upDistance, LRDistance, true);
  }

  aLeg.update();
  if(aLeg.cycleAt() == 4 || aLeg.cycleAt() == 5 || aLeg.cycleAt() == 6){
    mainKinematics(testHeightBACK-aLeg.heightAt(), aLeg.fbAt(), aLeg.lrAt(), aLeg.getMotor(),0,0,0);
  }else{
    mainKinematics(testHeightBACK-aLeg.heightAt(), aLeg.fbAt(), aLeg.lrAt(), aLeg.getMotor(),0,yRot,zRot);
  }
  
  cLeg.update();
  if(cLeg.cycleAt() == 4 || cLeg.cycleAt() == 5 || cLeg.cycleAt() == 6){
    mainKinematics(testHeight-cLeg.heightAt(), cLeg.fbAt(), cLeg.lrAt(), cLeg.getMotor(),0,0, 0);
  }else{
    mainKinematics(testHeight-cLeg.heightAt(), cLeg.fbAt(), cLeg.lrAt(), cLeg.getMotor(),0,yRot,zRot);
  }

  bLeg.update();
  if(bLeg.cycleAt() == 4 || bLeg.cycleAt() == 5 || bLeg.cycleAt() == 6){
    mainKinematics(testHeightBACK-bLeg.heightAt(), bLeg.fbAt(), bLeg.lrAt(), bLeg.getMotor(),0,0,0);
  }else{
    mainKinematics(testHeightBACK-bLeg.heightAt(), bLeg.fbAt(), bLeg.lrAt(), bLeg.getMotor(),0,yRot,zRot);
  }

  dLeg.update();
  if(dLeg.cycleAt() == 6 || dLeg.cycleAt() == 4 || dLeg.cycleAt() == 5){
    mainKinematics(testHeight-dLeg.heightAt(), dLeg.fbAt(), dLeg.lrAt(), dLeg.getMotor(),0,0,0);
  }else{
   mainKinematics(testHeight-dLeg.heightAt(), dLeg.fbAt(), dLeg.lrAt(), dLeg.getMotor(),0,yRot,zRot);
  }
}

void walk(rampLeg Leg, float timee, float backDistance, float upDistance, float LRDistance, bool d = true){
  float offTime = timee/6.0;

  if(Leg.cycleAt() == 0 && (Leg.allDone())){
    Leg.fbGo((backDistance / 4.0), (timee + (3*offTime)));
    Leg.lrGo((LRDistance / 4.0), (timee + (3*offTime)));
  }else if (Leg.cycleAt() == 1 && Leg.allDone())
  {
    Leg.fbGo(((2*backDistance)/4.0), timee);
    Leg.lrGo(((2*LRDistance)/4.0), timee);

  }else if (Leg.cycleAt() == 2 && Leg.allDone())
  {
    Leg.fbGo((3*backDistance/4.0),timee);
    Leg.lrGo((3*LRDistance/4.0),timee);
    Leg.hGo(0, timee);

  }else if (Leg.cycleAt() == 3 && Leg.allDone())
  {
    Leg.fbGo(backDistance, timee);
    Leg.lrGo(LRDistance, timee);
    Leg.hGo(0,timee);
  }
  else if(Leg.cycleAt() == 4 && Leg.allDone())
  {
    Leg.hGo(upDistance,offTime);
    Leg.fbGo(((backDistance) /4.0), offTime);
    Leg.lrGo(((LRDistance) /4.0), offTime);
  }else if(Leg.cycleAt() == 5 && Leg.allDone())
  {
    Leg.hGo(upDistance,offTime);
    Leg.fbGo((backDistance /4.0), offTime);
    Leg.lrGo((LRDistance /4.0), offTime);
  }else if(Leg.cycleAt() == 6 && Leg.allDone())
  {
    Leg.hGo(0,offTime);
    Leg.fbGo(0, offTime);
    Leg.lrGo(0, offTime);
  }

  if (Leg.allDone()){
    if (d){
     Leg.incCycle();
    }else{
      Leg.decCycle();
    }
  }
}