#include <Ramp.h>

#ifndef rampLegClass
#define rampLegClass
class rampLeg{
  private:
    int hip;
    rampLong  xH;
    rampLong xFB;
    rampLong xLR;
  public:
    rampLeg(int mhip){
        hip = mhip;
    }
    int getMotor(){
      return hip;
    }
    bool allDone(){
      if(xH.isFinished() && xFB.isFinished() && xLR.isFinished()){
        return true;
      }else{
        return false;
      }
    }
    void setPositions(float VH, float VLR, float VFB, float timee){
      xH.go(VH, timee);
      xFB.go(VFB, timee);
      xLR.go(VLR, timee);
    }
    void reset(){
      xH.go(0, 0);
      xFB.go(0, 0);
      xLR.go(0, 0);
    }
    void update(){
      xH.update();
      xLR.update();
      xFB.update();
    }
    void hGo(float position, float timee){
      xH.go(position,timee);
    }
    void fbGo(float position, float timee){
      xFB.go(position,timee);
    }
    void lrGo(float position, float timee){
      xLR.go(position,timee);
    }

    float heightAt(){
      return xH.getValue();
    }
    float fbAt(){
      return xFB.getValue();
    }
    float lrAt(){
      return xLR.getValue();
    }
    bool isGrounded(){
      if (xFB.getValue() == 0){
        return true;
      }else{
        return false;
      }
    }
};

#endif