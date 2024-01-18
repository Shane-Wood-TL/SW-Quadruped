#include <Arduino.h>
#include <Ramp.h>
#include <Adafruit_PWMServoDriver.h>
#include <TelnetStream.h>

#define A_HIP_OFFSET -10
#define A_KNEE_OFFSET 0
#define A_ANKLE_OFFSET 0

#define B_HIP_OFFSET 0
#define B_KNEE_OFFSET 15
#define B_ANKLE_OFFSET -15

#define C_HIP_OFFSET 0
#define C_KNEE_OFFSET 0
#define C_ANKLE_OFFSET 0

#define D_HIP_OFFSET 0
#define D_KNEE_OFFSET 10
#define D_ANKLE_OFFSET -5


//servo motor pwm limits
#define USMIN 771  // min value given from arduino lib
#define USMAX 2193 // max value given from arduino lib

//dimensions of the bot
#define yHalfDis 122
#define zHalfDis 190
// constant distances
#define aLength 95 // upper leg length more red
#define bLength 95 // lower leg length more purple
#define Ldis 50


#define aHip 3  //
#define aKnee 4  //
#define aAnkle 5 //

#define bHip 2 //
#define bKnee 1  //
#define bAnkle 0 //

#define cHip 2   //
#define cKnee 1  //
#define cAnkle 0 //

#define dHip 3    //
#define dKnee 4  //
#define dAnkle 5 //


//------------------------------------------------------------------------------------------------
//SCK 18
//MOSI 8
//MISO 10
//CE 16
//CSN 9


//------------------------------------------------------------------------------------------------
//values for non controlled turn
//current /default values
const float testHeightT = 120;
const float testHeightBACKT = 120;
const float testLRT = 0;
const float testFBT = 0;

//time for a cycle (in ms)
const float timeeT = 750;

//amount to change values by in a cycle
const float backDistanceT = 0; //(FB)
const float upDistanceT = -50; //xH
const float LRDistanceT =100; //xLR


//values for non controlled walk
const float testHeightW = 150;
const float testHeightBACKW = 130;
const float testLRW = 15;
const float testFBW = 0;

//time for a cycle (in ms)
const float timeeW = 100;

//amount to change values by in a cycle
const float backDistanceW = -60; //(FB)
const float upDistanceW = -40; //xH
const float LRDistanceW =0; //xLR


//------------------------------------------------------------------------------------------------
struct movementVariables{
 float testHeight; 
 float testHeightBACK;
 float testFB;
 float testLR;
 float upDistance;
 float backDistance;
 float LRDistance;
};


//------------------------------------------------------------------------------------------------
void populateStructs(movementVariables &walkV, movementVariables &turnV);
//(testHeightW, testHeightBACKW, testLRW, testFBW, upDistanceW, backDistanceW, LRDistanceW)


//------------------------------------------------------------------------------------------------
struct PayloadStruct {
  uint8_t eStop; //sw2
  uint8_t state;
  uint8_t gyro;
  uint8_t PID;
  int16_t j1_x;
  int16_t j1_y;
  int16_t j1_z;
  int16_t j2_x;
  int16_t j2_y;
  int16_t j2_z;
  uint8_t bt0_;
  uint8_t bt1_;
  uint8_t bt2_;
};


//------------------------------------------------------------------------------------------------
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


//------------------------------------------------------------------------------------------------
//legCtrls
void updateAll();
void resetAll();
bool allDone();


//------------------------------------------------------------------------------------------------
//Extra Math
float pytherm(float sidea, float sideb); //returns hypotenuse c
float raddec(float rad); //radians to degress
float loc(float a, float b, float c); // law of cosines, returns angle c in degress 
float pythermhypt(float sidea, float sidec); //returns side b
float decrad(float deg); //degrees to radians


//------------------------------------------------------------------------------------------------
//Movement
void walk(rampLeg &Leg, float timee, float backDistance, float upDistance, float LRDistance, bool d);
void WalkF(float yRot, float zRot, bool direction, float  testHeight, float testHeightBACK, float testFB, float testLR, float upDistance, float backDistance, float LRDistance);
void turn(float yRot, float zRot, bool clockwise, float  testHeight, float testHeightBACK, float testFB, float testLR, float upDistance, float backDistance, float LRDistance);


//------------------------------------------------------------------------------------------------
//Modes
void standing_0();
void IK_1(float xAngleV, float yAngleV, float zAngleV);
void FWalk_2(float yAngleV, float zAngleV);
void FTurn_3(float yAngleV, float zAngleV);
void User_4(float yAngleV, float zAngleV);

void Default_9();
void wakeup_9();


//------------------------------------------------------------------------------------------------
void getData();
void setCycle();


//------------------------------------------------------------------------------------------------
class Cords{
  public:
    float xH;
    float xFB;
    float xLR;
    float xRot;
    float yRot;
    float zRot;
    Cords(){
        xH = 0;
        xFB = 0;
        xLR = 0;
        xRot = 0;
        yRot = 0;
        zRot = 0;
    }
    void updateCords(float xHv, float xFBv, float xLRv, float xRotv, float yRotv, float zRotv){
        xH = xHv;
        xFB = xFBv;
        xLR = xLRv;
        xRot = xRotv;
        yRot = yRotv;
        zRot = zRotv;
    }
};


//------------------------------------------------------------------------------------------------
class motor{
  private:
    Adafruit_PWMServoDriver* pwm;
    float Hlimit;
    float Llimit;
    bool direction;
    float offset;
    int motorC;
  public:
  motor(Adafruit_PWMServoDriver* pwmV, int motorV, float LlimitV, float HlimitV, bool directionV, float offsetV){
    pwm = pwmV;
    motorC = motorV;
    Llimit = LlimitV;
    Hlimit = HlimitV;
    direction = directionV;
    offset = offsetV;
  }
    float Degree;
  void setDegree(float nDegree){
    if(nDegree+offset > Hlimit){
      nDegree = Hlimit-offset;
    }else if(nDegree+offset < Llimit){
      nDegree = Llimit-offset; 
    }else{
        nDegree = nDegree + offset;
    }
    if(direction){
        pwm->writeMicroseconds(motorC, (map(nDegree, 0, 180, USMIN, USMAX)));
    }else{
        pwm->writeMicroseconds(motorC, (map(nDegree, 180, 0, USMIN, USMAX)));
    }
  }
  int getMotor(){
    return motorC;
  }
};


//------------------------------------------------------------------------------------------------
class leg{
  motor* hip;
  motor* knee;
  motor* ankle;
  String name;
  public:
    leg(motor* hipV, motor* kneeV, motor* ankleV, String nameV){
      hip = hipV;
      knee = kneeV;
      ankle = ankleV;
      name = nameV;
    }

    void setAngles(float hipV, float kneeV, float ankleV){
      hip->setDegree(hipV);
      knee->setDegree(kneeV);
      ankle->setDegree(ankleV);
    }
    String getLegName(){
      return name;
    }
};


//------------------------------------------------------------------------------------------------
class kinematics{
  private:
  leg *legC;

  public:
  kinematics(leg* legV){
    legC = legV;
  }
  
  void mainKinematics(Cords position){
    //code to convert angles to change in positions
    float modZd;
    float modYd;
    float xHz;
    float xHy;
    float xFBz;
    float xLRy;

    xHz = zHalfDis * sin(decrad(position.zRot));
    modZd = zHalfDis * cos(decrad(position.zRot));

    if(legC->getLegName() == "C" || legC->getLegName() == "D"){
      position.xH -= xHz;
      position.xFB -= modZd - zHalfDis;
    }else{
      position.xH += xHz;
      position.xFB += modZd - zHalfDis;    
    }


    xHy = yHalfDis * sin(decrad(position.yRot));
    modYd = yHalfDis * cos(decrad(position.yRot));

  if(legC->getLegName() == "B" || legC->getLegName() == "D"){
    position.xH -= xHy;
    position.xLR -= modYd - yHalfDis;
  }else{
    position.xH += xHy;
    position.xLR += modYd - yHalfDis; 
  }

    TelnetStream.println();
    TelnetStream.print(position.xH);
    TelnetStream.print("    ");
    TelnetStream.print(position.xLR);
    TelnetStream.print("    ");
    TelnetStream.println(position.xFB);

    if(position.xH > 134){
      position.xH = 134;
    }

    float innerLegLength = pytherm(position.xH, Ldis + position.xLR);
    float modLegL = pythermhypt(Ldis, innerLegLength);
    float innerAngleA = acos(position.xH / innerLegLength);
    innerAngleA = raddec(innerAngleA);
    float innerAngleB = acos(Ldis / innerLegLength);
    innerAngleB = raddec(innerAngleB);
    float modLegLL = pytherm(position.xFB, modLegL);
    float innerAngleKneeA = acos(modLegL / modLegLL);
    innerAngleKneeA = raddec(innerAngleKneeA);
    float outerAngleKneeB = loc(aLength, modLegLL, bLength);
    float kneeAngle = loc(aLength, bLength, modLegLL);
    if (position.xFB <= 0){
      legC->setAngles(innerAngleA + innerAngleB, 90-(outerAngleKneeB + innerAngleKneeA), kneeAngle);
    }else{
      legC->setAngles(innerAngleA + innerAngleB, 90-(outerAngleKneeB-innerAngleKneeA), kneeAngle);
    }
    
  }
};