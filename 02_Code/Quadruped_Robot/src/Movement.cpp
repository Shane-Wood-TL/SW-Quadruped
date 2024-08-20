#include <Arduino.h>
#include <externFunctions.h>

//------------------------------------------------------------------------------------------------
extern rampLeg aLegR;
extern rampLeg bLegR;
extern rampLeg cLegR;
extern rampLeg dLegR;

extern Cords aCords;
extern Cords bCords;
extern Cords cCords;
extern Cords dCords;

extern kinematics AlegK;
extern kinematics BlegK;
extern kinematics ClegK;
extern kinematics DlegK;


//------------------------------------------------------------------------------------------------
//create pointers to legs
rampLeg* aLegAdd = &aLegR;
rampLeg* bLegAdd = &bLegR;
rampLeg* cLegAdd = &cLegR;
rampLeg* dLegAdd = &dLegR;

//------------------------------------------------------------------------------------------------
void WalkF(float yRot, float zRot, bool direction, float  testHeight, float testHeightBACK, float testFB, float testLR, float upDistance, float backDistance, float LRDistance){
  float timee = 45; //45
  timee = timee;
  updateAll();

  if(allDone()){
    walk(*aLegAdd, timee, backDistance, upDistance, LRDistance, direction);
    walk(*cLegAdd, timee, backDistance, upDistance, LRDistance, direction);
    walk(*dLegAdd, timee, backDistance, upDistance, LRDistance, direction);
    walk(*bLegAdd, timee, backDistance, upDistance, LRDistance, direction);
    //Serial.println("all done");
  }

  
  if(!aLegR.isGrounded()){
    aCords.updateCords(testHeightBACK+aLegR.heightAt(), testFB+aLegR.fbAt(), testLR-aLegR.lrAt(),0,0,0);
    AlegK.mainKinematics(aCords);
  }else{
    aCords.updateCords(testHeightBACK+aLegR.heightAt(), testFB+aLegR.fbAt(), testLR-aLegR.lrAt(),0,yRot,zRot);
    AlegK.mainKinematics(aCords);
  }
  
  
  if(!cLegR.isGrounded()){
    cCords.updateCords(testHeightBACK+cLegR.heightAt(), testFB+cLegR.fbAt(), testLR-cLegR.lrAt(),0,0,0);
    ClegK.mainKinematics(cCords);
  }else{
    cCords.updateCords(testHeightBACK+cLegR.heightAt(), testFB+cLegR.fbAt(), testLR-cLegR.lrAt(),0,yRot,zRot);
    ClegK.mainKinematics(cCords);
  }

  
  if(!bLegR.isGrounded()){
    bCords.updateCords(testHeightBACK+bLegR.heightAt(), testFB+bLegR.fbAt(), testLR-bLegR.lrAt(),0,0,0);
    BlegK.mainKinematics(bCords);
  }else{
    bCords.updateCords(testHeightBACK+bLegR.heightAt(), testFB+bLegR.fbAt(), testLR-bLegR.lrAt(),0,yRot,zRot);
    BlegK.mainKinematics(bCords);
  }

  
  if(!dLegR.isGrounded()){
    dCords.updateCords(testHeightBACK+dLegR.heightAt(), testFB+dLegR.fbAt(), testLR-dLegR.lrAt(),0,0,0);
    DlegK.mainKinematics(dCords);
  }else{
    dCords.updateCords(testHeightBACK+dLegR.heightAt(), testFB+dLegR.fbAt(), testLR-dLegR.lrAt(),0,yRot,zRot);
    DlegK.mainKinematics(dCords);
  }
}


//------------------------------------------------------------------------------------------------
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
    if(!aLegR.isGrounded()){
      aCords.updateCords(testHeightBACK+aLegR.heightAt(), testFB, testLR-aLegR.lrAt(),0,0,0);
      AlegK.mainKinematics(aCords);
    }else{
      aCords.updateCords(testHeightBACK+aLegR.heightAt(), testFB, testLR-aLegR.lrAt(),0,yRot,zRot);
      AlegK.mainKinematics(aCords);
    }
    
    
    if(!cLegR.isGrounded()){
      cCords.updateCords(testHeightBACK+cLegR.heightAt(), testFB-cLegR.fbAt(), testLR,0,0,0);
      ClegK.mainKinematics(cCords);
    }else{
      cCords.updateCords(testHeightBACK+aLegR.heightAt(), testFB-cLegR.fbAt(), testLR,0,0,0);
      ClegK.mainKinematics(cCords);
    }

    
    if(!bLegR.isGrounded()){
      bCords.updateCords(testHeightBACK+bLegR.heightAt(), testFB+bLegR.fbAt(), testLR,0,0,0);
      BlegK.mainKinematics(bCords);
    }else{
      bCords.updateCords(testHeightBACK+bLegR.heightAt(), testFB+bLegR.fbAt(), testLR,0,0,0);
      BlegK.mainKinematics(bCords);
    }

    
    if(!dLegR.isGrounded()){
      dCords.updateCords(testHeightBACK+dLegR.heightAt(), testFB, testLR-dLegR.lrAt(),0,0,0);
      DlegK.mainKinematics(dCords);
    }else{
      dCords.updateCords(testHeightBACK+dLegR.heightAt(), testFB, testLR-dLegR.lrAt(),0,yRot,zRot);
      DlegK.mainKinematics(dCords);
    }
    
  }else{

   if(!aLegR.isGrounded()){
      aCords.updateCords(testHeightBACK+aLegR.heightAt(), testFB, testLR+aLegR.lrAt(),0,0,0);
      AlegK.mainKinematics(aCords);
    }else{
      aCords.updateCords(testHeightBACK+aLegR.heightAt(), testFB, testLR+aLegR.lrAt(),0,yRot,zRot);
      AlegK.mainKinematics(aCords);
    }

    
    
    if(!cLegR.isGrounded()){
      cCords.updateCords(testHeightBACK+cLegR.heightAt(), testFB+cLegR.fbAt(), testLR,0,0,0);
      ClegK.mainKinematics(cCords);
    }else{
      cCords.updateCords(testHeightBACK+cLegR.heightAt(), testFB+cLegR.fbAt(), testLR,0,yRot,zRot);
      ClegK.mainKinematics(cCords);
    }

    
    if(!bLegR.isGrounded()){
      bCords.updateCords(testHeightBACK+bLegR.heightAt(), testFB-bLegR.fbAt(), testLR,0,0,0);
      BlegK.mainKinematics(bCords);
    }else{
      bCords.updateCords(testHeightBACK+bLegR.heightAt(), testFB-bLegR.fbAt(), testLR,0,yRot,zRot);
      BlegK.mainKinematics(bCords);
    }


    
    if(!dLegR.isGrounded()){
      dCords.updateCords(testHeightBACK+dLegR.heightAt(), testFB, testLR+dLegR.lrAt(),0,0,0);
      DlegK.mainKinematics(dCords);
    }else{
      dCords.updateCords(testHeightBACK+dLegR.heightAt(), testFB, testLR+dLegR.lrAt(),0,yRot,zRot);
      DlegK.mainKinematics(dCords);
    }
  }
}


//------------------------------------------------------------------------------------------------
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


//------------------------------------------------------------------------------------------------
void stop(float timee){
  aLegR.setPositions(0,0,0,timee);
  bLegR.setPositions(0,0,0,timee);
  cLegR.setPositions(0,0,0,timee);
  dLegR.setPositions(0,0,0,timee);
  aLegR.setCycle(0);
  bLegR.setCycle(0);
  cLegR.setCycle(0);
  dLegR.setCycle(0);
}


//------------------------------------------------------------------------------------------------
void populateStructs(movementVariables &walkV, movementVariables &turnV){
  walkV.testHeight = testHeightW;
  walkV.testHeightBACK = testHeightBACKW;
  walkV.testFB = testFBW;
  walkV.testLR = testLRW;
  walkV.upDistance = upDistanceW;
  walkV.backDistance = backDistanceW;
  walkV.LRDistance = LRDistanceW;

  turnV.testHeight = testHeightW;
  turnV.testHeightBACK = testHeightBACKW;
  turnV.testFB = testFBW;
  turnV.testLR = testLRW;
  turnV.upDistance = upDistanceW;
  turnV.backDistance = backDistanceW;
  turnV.LRDistance = LRDistanceW;
}