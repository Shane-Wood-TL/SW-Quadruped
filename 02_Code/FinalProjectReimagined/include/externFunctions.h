#include <Arduino.h>
#include <Ramp.h>


//class
class rampLeg{
  private:
    int hip;
    ramp xH;
    ramp xFB;
    ramp xLR;
    int cycle;
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
    void setCycle(int newCycle){
      cycle = newCycle;
    }
    int cycleAt(){
      return cycle;
    }
};


//Kinematics
void mainKinematics(float xH , float xFB, float xLR, int hipMotor, float xRot, float yRot, float zRot);
void all_90s();

//legCtrls
void updateAll();
void resetAll();

//Extra Math
float pytherm(float sidea, float sideb); //returns hypotenuse c
float raddec(float rad); //radians to degress
float loc(float a, float b, float c); // law of cosines, returns angle c in degress 
float pythermhypt(float sidea, float sidec); //returns side b
float decrad(float deg); //degrees to radians


//walking
void walk(rampLeg Leg, float timee, float backDistance, float upDistance, float LRDistance, bool d);
void WalkF(float yRot, float zRot);



