#include <Arduino.h>
#include <externFunctions.h>

//Ramps
extern rampLeg aLegR;
extern rampLeg bLegR;
extern rampLeg cLegR;
extern rampLeg dLegR;


//sets all legs to 0
void resetAll(){ 
  aLegR.reset();
  bLegR.reset();
  cLegR.reset();
  dLegR.reset();
}

//updates all ramps
void updateAll(){
 aLegR.update();
 bLegR.update();
 cLegR.update();
 dLegR.update();
}

//checks if all ramps are done
bool allDone(){
  return aLegR.allDone() && bLegR.allDone() && cLegR.allDone() && dLegR.allDone();
}
