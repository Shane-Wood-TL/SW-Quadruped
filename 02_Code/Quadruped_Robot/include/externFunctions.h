#include <Arduino.h>
#include <Ramp.h>


//SCK 18
//MOSI 8
//MISO 10
//CE 16
//CSN 9




//values for non controlled turn
//current /default values
const float testHeightT = 150;
const float testHeightBACKT = 150;
const float testLRT = 0;
const float testFBT = 0;

//time for a cycle (in ms)
const float timeeT = 100;

//amount to change values by in a cycle
const float backDistanceT = 0; //(FB)
const float upDistanceT = -50; //xH
const float LRDistanceT =100; //xLR


//values for non controlled walk
const float testHeightW = 150;
const float testHeightBACKW = 170;
const float testLRW = 0;
const float testFBW = 0;

//time for a cycle (in ms)
const float timeeW = 750;

//amount to change values by in a cycle
const float backDistanceW = -50; //(FB)
const float upDistanceW = -70; //xH
const float LRDistanceW =0; //xLR

struct movementVariables{
 float testHeight; 
 float testHeightBACK;
 float testFB;
 float testLR;
 float upDistance;
 float backDistance;
 float LRDistance;
};

void populateStructs(movementVariables &walkV, movementVariables &turnV);
//(testHeightW, testHeightBACKW, testLRW, testFBW, upDistanceW, backDistanceW, LRDistanceW)


struct PayloadStruct {
  uint8_t eStop; //sw2
  uint8_t state;
  uint8_t gyro;
  uint8_t PID;
  int16_t j1_x;
  int16_t j1_y;
  int16_t j2_x;
  int16_t j2_y;
  uint8_t j1_b;
  uint8_t j2_b;
};

//class
class rampLeg{
  private:
    int hip;
    rampLong  xH;
    rampLong xFB;
    rampLong xLR;
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
    bool isGrounded(){
      if (xFB.getValue() == 0){
        return true;
      }else{
        return false;
      }
    }
};

//Kinematics
void mainKinematics(float xH , float xFB, float xLR, int hipMotor, float xRot, float yRot, float zRot);
void all_90s();

//legCtrls
void updateAll();
void resetAll();
bool allDone();

//Extra Math
float pytherm(float sidea, float sideb); //returns hypotenuse c
float raddec(float rad); //radians to degress
float loc(float a, float b, float c); // law of cosines, returns angle c in degress 
float pythermhypt(float sidea, float sidec); //returns side b
float decrad(float deg); //degrees to radians

//Movement
void walk(rampLeg &Leg, float timee, float backDistance, float upDistance, float LRDistance, bool d);
void WalkF(float yRot, float zRot, bool direction, float  testHeight, float testHeightBACK, float testFB, float testLR, float upDistance, float backDistance, float LRDistance);
void turn(float yRot, float zRot, bool clockwise, float  testHeight, float testHeightBACK, float testFB, float testLR, float upDistance, float backDistance, float LRDistance);


//Modes
void standing_0();
void IK_1();
void FWalk_2();
void FTurn_3();
void User_4();


void Default_9();
void wakeup_9();


void getData();

void setCycle();