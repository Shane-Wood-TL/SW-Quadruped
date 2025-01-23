

#ifndef rampLegClass
#define rampLegClass
#include "../interpolation.h"
class rampLeg{
  private:
    uint8_t hip;
    interpolation  xH;
    interpolation xFB;
    interpolation xLR;
    uint8_t cycle;
  public:
    rampLeg(uint8_t mhip) : hip(mhip), xH(0), xFB(0), xLR(0) {
        hip = mhip;
    }
    int getMotor(){
      return hip;
    }
    bool allDone(){
      if(xH.is_complete() && xFB.is_complete() && xLR.is_complete()){
        return true;
      }else{
        return false;
      }
    }
    void setPositions(float VH, float VLR, float VFB, float timee){
      xH.set_target(VH, timee);
      xFB.set_target(VFB, timee);
      xLR.set_target(VLR, timee);
    }
    void reset(){
      xH.set_target(0, 0);
      xFB.set_target(0, 0);
      xLR.set_target(0, 0);
    }
    void update(){
      xH.get_position();
      xLR.get_position();
      xFB.get_position();
    }
    void incCycle(){
      cycle++;
      if (cycle > 6){
        cycle = 0;
      }
    }
    void decCycle(){
      cycle--;
      if (cycle < 0){
        cycle = 6;
      }
    }
    void hGo(float position, float timee){
      xH.set_target(position,timee);
    }
    void fbGo(float position, float timee){
      xFB.set_target(position,timee);
    }
    void lrGo(float position, float timee){
      xLR.set_target(position,timee);
    }

    float heightAt(){
      return xH.get_position();
    }
    float fbAt(){
      return xFB.get_position();
    }
    float lrAt(){
      return xLR.get_position();
    }
    void setCycle(int newCycle){
      cycle = newCycle;
    }
     int cycleAt(){
      return cycle;
    }
    bool isGrounded(){
      if (xFB.get_position() == 0){
        return true;
      }else{
        return false;
      }
    }
};

#endif