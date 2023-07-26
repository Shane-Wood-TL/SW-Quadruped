#include <Arduino.h>
#include <Ramp.h>

void walk(int &Cycle, ramp &FB, ramp &Height, float timee, float backDistance, float upDistance){
  float offTime = timee/6.0;
  //Serial.print(Cycle);
  // Serial.println(FB.getTarget());
  if(Cycle == 0 && (FB.isFinished())){
    // Serial.print(Cycle);
    // Serial.println(FB.isFinished());

    FB.go((backDistance / 4.0), (timee + (3*offTime)));
    Cycle = 1;

  }else if (Cycle == 1 && FB.isFinished())
  {
    // Serial.print(Cycle);
    // Serial.println(FB.isFinished());
    FB.go(((2*backDistance)/4.0), timee);
    Cycle = 2;

  }else if (Cycle == 2 && FB.isFinished())
  {
    // Serial.print(Cycle);
    // Serial.println(FB.isFinished());
    FB.go((3*backDistance/4.0),timee);
    Height.go(0, timee);
    Cycle = 3;

  }else if (Cycle == 3 && FB.isFinished())
  {
    // Serial.print(Cycle);
    // Serial.println(FB.isFinished());
    FB.go(backDistance, timee);
    Height.go(0,timee);
    
    Cycle = 4;

  }
  else if(Cycle == 4 && FB.isFinished())
  {
    // Serial.print(Cycle);
    // Serial.println(FB.isFinished());
    Height.go(upDistance,offTime);
    FB.go(((backDistance) /4.0), offTime);
    Cycle = 5;
  }  else if(Cycle == 5 && FB.isFinished())
  {
    // Serial.print(Cycle);
    // Serial.println(FB.isFinished());
    Height.go(upDistance,offTime);
    FB.go((backDistance /4.0), offTime);
    Cycle = 6;
  }  else if(Cycle == 6 && FB.isFinished())
  {
    // Serial.print(Cycle);
    // Serial.println(FB.isFinished());
    Height.go(0,offTime);
    FB.go(0, offTime);
    Cycle = 0;
  }

}