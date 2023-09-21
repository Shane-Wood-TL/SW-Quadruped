#include <Arduino.h>
#include <externFunctions.h>


extern rampLeg aLeg;
extern rampLeg bLeg;
extern rampLeg cLeg;
extern rampLeg dLeg;

//create pointers to legs
rampLeg* aLegAdd = &aLeg;
rampLeg* bLegAdd = &bLeg;
rampLeg* cLegAdd = &cLeg;
rampLeg* dLegAdd = &dLeg;


void WalkF(float yRot, float zRot, bool direction, float  testHeight, float testHeightBACK, float testFB, float testLR, float upDistance, float backDistance, float LRDistance){
  float timee = sqrt(pow(backDistance,2) + pow(upDistance,2) + pow(LRDistance,2));
  timee = timee;
  updateAll();

  if(allDone()){
    walk(*aLegAdd, timee, backDistance, upDistance, LRDistance, direction);
    walk(*cLegAdd, timee, backDistance, upDistance, LRDistance, direction);
    walk(*dLegAdd, timee, backDistance, upDistance, LRDistance, direction);
    walk(*bLegAdd, timee, backDistance, upDistance, LRDistance, direction);
    //Serial.println("all done");
  }

  
  if(!aLeg.isGrounded()){
    mainKinematics(testHeightBACK+aLeg.heightAt(), testFB+aLeg.fbAt(), testLR-aLeg.lrAt(), aLeg.getMotor(),0,0,0);
  }else{
    mainKinematics(testHeightBACK+aLeg.heightAt(), testFB+aLeg.fbAt(), testLR-aLeg.lrAt(), aLeg.getMotor(),0,yRot,zRot);
  }
  
  
  if(!cLeg.isGrounded()){
    mainKinematics(testHeight+cLeg.heightAt(), testFB+cLeg.fbAt(), testLR-cLeg.lrAt(), cLeg.getMotor(),0,0, 0);
  }else{
    mainKinematics(testHeight+cLeg.heightAt(), testFB+cLeg.fbAt(), testLR-cLeg.lrAt(), cLeg.getMotor(),0,yRot,zRot);
  }

  
  if(!bLeg.isGrounded()){
    mainKinematics(testHeightBACK+bLeg.heightAt(), testFB+bLeg.fbAt(), testLR+bLeg.lrAt(), bLeg.getMotor(),0,0,0);
  }else{
    mainKinematics(testHeightBACK+bLeg.heightAt(), testFB+bLeg.fbAt(), testLR+bLeg.lrAt(), bLeg.getMotor(),0,yRot,zRot);
  }

  
  if(!dLeg.isGrounded()){
    mainKinematics(testHeight+dLeg.heightAt(), testFB+dLeg.fbAt(), testLR+dLeg.lrAt(), dLeg.getMotor(),0,0,0);
  }else{
   mainKinematics(testHeight+dLeg.heightAt(), testFB+dLeg.fbAt(), testLR+dLeg.lrAt(), dLeg.getMotor(),0,yRot,zRot);
  }
}


void turn(float yRot, float zRot, bool clockwise, float  testHeight, float testHeightBACK, float testFB, float testLR, float upDistance, float backDistance, float LRDistance){
  float timee = sqrt(pow(backDistance,2) + pow(upDistance,2) + pow(LRDistance,2));
  timee = timee;
  updateAll();
  if(allDone()){
    walk(*aLegAdd, timee, backDistance, upDistance, LRDistance, true);
    walk(*cLegAdd, timee, backDistance, upDistance, LRDistance, true);
    walk(*dLegAdd, timee, backDistance, upDistance, LRDistance, true);
    walk(*bLegAdd, timee, backDistance, upDistance, LRDistance, true);
    }
  if(clockwise){  
    if(!aLeg.isGrounded()){
      mainKinematics(testHeightBACK+aLeg.heightAt(), testFB, testLR-aLeg.lrAt(), aLeg.getMotor(),0,0,0);
    }else{
      mainKinematics(testHeightBACK+aLeg.heightAt(), testFB, testLR-aLeg.lrAt(), aLeg.getMotor(),0,yRot,zRot);
    }
    
    
    if(!cLeg.isGrounded()){
      mainKinematics(testHeight+cLeg.heightAt(), testFB-cLeg.fbAt(), testLR, cLeg.getMotor(),0,0, 0);
    }else{
      mainKinematics(testHeight+cLeg.heightAt(), testFB-cLeg.fbAt(), testLR, cLeg.getMotor(),0,yRot,zRot);
    }

    
    if(!bLeg.isGrounded()){
      mainKinematics(testHeightBACK+bLeg.heightAt(), testFB+bLeg.fbAt(), testLR, bLeg.getMotor(),0,0,0);
    }else{
      mainKinematics(testHeightBACK+bLeg.heightAt(), testFB+bLeg.fbAt(), testLR, bLeg.getMotor(),0,yRot,zRot);
    }

    
    if(!dLeg.isGrounded()){
      mainKinematics(testHeight+dLeg.heightAt(), testFB, testLR-dLeg.lrAt(), dLeg.getMotor(),0,0,0);
    }else{
    mainKinematics(testHeight+dLeg.heightAt(), testFB, testLR-dLeg.lrAt(), dLeg.getMotor(),0,yRot,zRot);
    }
    
  }else{

   if(!aLeg.isGrounded()){
      mainKinematics(testHeightBACK+aLeg.heightAt(), testFB, testLR+aLeg.lrAt(), aLeg.getMotor(),0,0,0);
    }else{
      mainKinematics(testHeightBACK+aLeg.heightAt(), testFB, testLR+aLeg.lrAt(), aLeg.getMotor(),0,yRot,zRot);
    }
    
    
    if(!cLeg.isGrounded()){
      mainKinematics(testHeight+cLeg.heightAt(), testFB+cLeg.fbAt(), testLR, cLeg.getMotor(),0,0, 0);
    }else{
      mainKinematics(testHeight+cLeg.heightAt(), testFB+cLeg.fbAt(), testLR, cLeg.getMotor(),0,yRot,zRot);
    }

    
    if(!bLeg.isGrounded()){
      mainKinematics(testHeightBACK+bLeg.heightAt(), testFB-bLeg.fbAt(), testLR, bLeg.getMotor(),0,0,0);
    }else{
      mainKinematics(testHeightBACK+bLeg.heightAt(), testFB-bLeg.fbAt(), testLR, bLeg.getMotor(),0,yRot,zRot);
    }

    
    if(!dLeg.isGrounded()){
      mainKinematics(testHeight+dLeg.heightAt(), testFB, testLR+dLeg.lrAt(), dLeg.getMotor(),0,0,0);
    }else{
    mainKinematics(testHeight+dLeg.heightAt(), testFB, testLR+dLeg.lrAt(), dLeg.getMotor(),0,yRot,zRot);
    }
  }
}



void walk(rampLeg &Leg, float timee, float backDistance, float upDistance, float LRDistance, bool d){
  float offTime = timee/6.0;
  if(Leg.cycleAt() == 0 && (Leg.allDone())){
    Leg.fbGo((backDistance / 4.0), (timee + (3*offTime)));
    Leg.lrGo((LRDistance / 4.0), (timee + (3*offTime)));
    if (d){
     Leg.incCycle();
    }else{
      Leg.decCycle();
    }
  }else if (Leg.cycleAt() == 1 && Leg.allDone())
  {
    Leg.fbGo(((2*backDistance)/4.0), timee);
    Leg.lrGo(((2*LRDistance)/4.0), timee);
    if (d){
     Leg.incCycle();
    }else{
      Leg.decCycle();
    }

  }else if (Leg.cycleAt() == 2 && Leg.allDone())
  {
    Leg.fbGo((3*backDistance/4.0),timee);
    Leg.lrGo((3*LRDistance/4.0),timee);
    Leg.hGo(0, timee);
    if (d){
     Leg.incCycle();
    }else{
      Leg.decCycle();
    }

  }else if (Leg.cycleAt() == 3 && Leg.allDone())
  {
    Leg.fbGo(backDistance, timee);
    Leg.lrGo(LRDistance, timee);
    Leg.hGo(0,timee);
    if (d){
     Leg.incCycle();
    }else{
      Leg.decCycle();
    }
  }
  else if(Leg.cycleAt() == 4 && Leg.allDone())
  {
    Leg.hGo(upDistance,offTime);
    Leg.fbGo(((backDistance) /4.0), offTime);
    Leg.lrGo(((LRDistance) /4.0), offTime);
    if (d){
     Leg.incCycle();
    }else{
      Leg.decCycle();
    }
  }else if(Leg.cycleAt() == 5 && Leg.allDone())
  {
    Leg.hGo(upDistance,offTime);
    Leg.fbGo((backDistance /4.0), offTime);
    Leg.lrGo((LRDistance /4.0), offTime);
    if (d){
     Leg.incCycle();
    }else{
      Leg.decCycle();
    }
  }else if(Leg.cycleAt() == 6 && Leg.allDone())
  {
    Leg.hGo(0,offTime);
    Leg.fbGo(0, offTime);
    Leg.lrGo(0, offTime);
    if (d){
     Leg.incCycle();
    }else{
      Leg.decCycle();
    }
  }
}

void stop(float timee){
  aLeg.setPositions(0,0,0,timee);
  bLeg.setPositions(0,0,0,timee);
  cLeg.setPositions(0,0,0,timee);
  dLeg.setPositions(0,0,0,timee);
  aLeg.setCycle(0);
  bLeg.setCycle(0);
  cLeg.setCycle(0);
  dLeg.setCycle(0);
}

