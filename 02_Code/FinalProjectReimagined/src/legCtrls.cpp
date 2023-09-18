#include <Arduino.h>
#include <externFunctions.h>

//Ramps
extern rampLeg aLeg;
extern rampLeg bLeg;
extern rampLeg cLeg;
extern rampLeg dLeg;


void resetAll(){ 
  aLeg.reset();
  bLeg.reset();
  cLeg.reset();
  dLeg.reset();
}

void updateAll(){
 aLeg.update();
 bLeg.update();
 cLeg.update();
 dLeg.update();
}

bool allDone(){
  return aLeg.allDone() && bLeg.allDone() && cLeg.allDone() && dLeg.allDone();
}
