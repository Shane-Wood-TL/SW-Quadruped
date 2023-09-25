#include <Arduino.h>
#include <externFunctions.h>

//Ramps
extern rampLeg aLeg;
extern rampLeg bLeg;
extern rampLeg cLeg;
extern rampLeg dLeg;


//sets all legs to 0
void resetAll(){ 
  aLeg.reset();
  bLeg.reset();
  cLeg.reset();
  dLeg.reset();
}

//updates all ramps
void updateAll(){
 aLeg.update();
 bLeg.update();
 cLeg.update();
 dLeg.update();
}

//checks if all ramps are done
bool allDone(){
  return aLeg.allDone() && bLeg.allDone() && cLeg.allDone() && dLeg.allDone();
}
