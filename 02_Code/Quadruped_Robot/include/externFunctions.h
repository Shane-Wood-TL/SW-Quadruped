#include <Arduino.h>
#include <Ramp.h>
#include <Adafruit_PWMServoDriver.h>


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


//------------------------------------------------------------------------------------------------
//Extra Math
float pytherm(float sidea, float sideb); //returns hypotenuse c
float raddec(float rad); //radians to degress
float loc(float a, float b, float c); // law of cosines, returns angle c in degress 
float pythermhypt(float sidea, float sidec); //returns side b
float decrad(float deg); //degrees to radians


//------------------------------------------------------------------------------------------------
//Movement
//void walk(rampLeg &Leg, float timee, float backDistance, float upDistance, float LRDistance, bool d);
//void WalkF(float yRot, float zRot, bool direction, float  testHeight, float testHeightBACK, float testFB, float testLR, float upDistance, float backDistance, float LRDistance);
//void turn(float yRot, float zRot, bool clockwise, float  testHeight, float testHeightBACK, float testFB, float testLR, float upDistance, float backDistance, float LRDistance);


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
//void setCycle();


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
    void updatePosition(float xHv, float xFBv, float xLRv){
        xH = xHv;
        xFB = xFBv;
        xLR = xLRv;
    }
};


//------------------------------------------------------------------------------------------------
class motor{
  private:
    Adafruit_PWMServoDriver* pwm;
    float Hlimit;
    float Llimit;
    bool direction;
    float* offset;
    int motorC;
  public:
  motor(Adafruit_PWMServoDriver* pwmV, int motorV, float LlimitV, float HlimitV, bool directionV, float* offsetV){
    pwm = pwmV;
    motorC = motorV;
    Llimit = LlimitV;
    Hlimit = HlimitV;
    direction = directionV;
    offset = offsetV;
  }
    float Degree;
  void setDegree(float nDegree){
    if(nDegree+*offset > Hlimit){
      nDegree = Hlimit-*offset;
    }else if(nDegree+*offset < Llimit){
      nDegree = Llimit-*offset; 
    }else{
        nDegree = nDegree + *offset;
    }
    if(direction){
        //pwm->writeMicroseconds(motorC, (map(nDegree, 0, 180, USMIN, USMAX)));
    }else{
        //pwm->writeMicroseconds(motorC, (map(nDegree, 180, 0, USMIN, USMAX)));
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




struct singleCycle {
    float legPositions[12];
    singleCycle(float AxH, float AxLR, float AxFB,
                float BxH, float BxLR, float BxFB,
                float CxH, float CxLR, float CxFB,
                float DxH, float DxLR, float DxFB) {
        
        legPositions[0] = AxH;
        legPositions[1] = AxLR;
        legPositions[2] = AxFB;

        legPositions[3] = BxH;
        legPositions[4] = BxLR;
        legPositions[5] = BxFB;

        legPositions[6] = CxH;
        legPositions[7] = CxLR;
        legPositions[8] = CxFB;

        legPositions[9] = DxH;
        legPositions[10] = DxLR;
        legPositions[11] = DxFB;
    }
    void resetPositions(){
        legPositions[0] = 0;
        legPositions[1] = 0;
        legPositions[2] = 0;

        legPositions[3] = 0;
        legPositions[4] = 0;
        legPositions[5] = 0;

        legPositions[6] = 0;
        legPositions[7] = 0;
        legPositions[8] = 0;

        legPositions[9] = 0;
        legPositions[10] = 0;
        legPositions[11] = 0;
    }
    void setOffsets(singleCycle newOffsets){
      
    }
};

class movementCycles{
    
    public:
    int currentCycleIndex;
    singleCycle* cycle;
    float totalTime;
    int cycleCount;
    bool absolutePositioning;
    bool direction;
    movementCycles(int cycleCount, bool absolutePositioning, bool direction, float totalTime, singleCycle* cycle)
        : cycleCount(cycleCount), absolutePositioning(absolutePositioning), direction(direction), totalTime(totalTime), cycle(cycle) {
            currentCycleIndex = 0;
        }

    void nextCycle(){
    
        if(currentCycleIndex >= cycleCount-1){
            currentCycleIndex = 0;
        }else{
            currentCycleIndex++;
        }
    }
    float *getPositons(){
        return cycle[currentCycleIndex].legPositions;
    }
};

class cycleControl{
  public:
    movementCycles *activeCycle;

    rampLeg *AlegR;
    rampLeg *BlegR;
    rampLeg *ClegR;
    rampLeg *DlegR;

    kinematics *AlegK;
    kinematics *BlegK;
    kinematics *ClegK;
    kinematics *DlegK;

    Cords *aCords;
    Cords *bCords;
    Cords *cCords;
    Cords *dCords;



    int currentStage = 0;
    
    cycleControl(movementCycles *activeCycle, 
    rampLeg *AlegR, rampLeg *BlegR, rampLeg *ClegR, rampLeg *DlegR, 
    kinematics *AlegK,kinematics *BlegK, kinematics *ClegK, kinematics *DlegK, 
    Cords *aCords, Cords *bCords, Cords *cCords, Cords *dCords) : 
    activeCycle(activeCycle),
    AlegR(AlegR), BlegR(BlegR), ClegR(ClegR), DlegR(DlegR),
    AlegK(AlegK), BlegK(BlegK), ClegK(ClegK), DlegK(DlegK),
    aCords(aCords), bCords(bCords), cCords(cCords), dCords(dCords) {
      AlegR->reset();
      BlegR->reset();
      ClegR->reset();
      DlegR->reset();
    }

    void updateRampPositions(){
        //update ramps
        AlegR->update();
        BlegR->update();
        ClegR->update();
        DlegR->update();
    }

    void checkCycle(){
        if(AlegR->allDone() && BlegR->allDone() && ClegR->allDone() && DlegR->allDone()){
            //go to next cycle if done
            activeCycle->nextCycle();
            float cycleTime = activeCycle->totalTime/activeCycle->cycleCount;
            //set new positons and time
            float* positions = activeCycle->getPositons();
            AlegR->hGo(positions[0],cycleTime);
            AlegR->lrGo(positions[1],cycleTime);
            AlegR->fbGo(positions[2],cycleTime);

            BlegR->hGo(positions[3],cycleTime);
            BlegR->lrGo(positions[4],cycleTime);
            BlegR->fbGo(positions[5],cycleTime);

            ClegR->hGo(positions[6],cycleTime);
            ClegR->lrGo(positions[7],cycleTime);
            ClegR->fbGo(positions[8],cycleTime);

            DlegR->hGo(positions[9],cycleTime);
            DlegR->lrGo(positions[10],cycleTime);
            DlegR->fbGo(positions[11],cycleTime);
        }
    }

    void setLegPositions(){
        AlegK->mainKinematics(*aCords);
        BlegK->mainKinematics(*bCords);
        ClegK->mainKinematics(*cCords);
        DlegK->mainKinematics(*dCords);
    }

    void continueCycle(Cords AcurrentPosition, Cords BcurrentPosition,Cords CcurrentPosition,Cords DcurrentPosition){
        checkCycle();

        updateRampPositions();

        if(activeCycle->absolutePositioning){
            aCords->updatePosition(AlegR->heightAt(),AlegR->fbAt(),AlegR->lrAt());
            bCords->updatePosition(BlegR->heightAt(),BlegR->fbAt(),BlegR->lrAt());
            cCords->updatePosition(ClegR->heightAt(),ClegR->fbAt(),ClegR->lrAt());
            dCords->updatePosition(DlegR->heightAt(),DlegR->fbAt(),DlegR->lrAt());

        }else{
            aCords->updatePosition(AcurrentPosition.xH+AlegR->heightAt(),AcurrentPosition.xFB+AlegR->fbAt(),AcurrentPosition.xLR+AlegR->lrAt());
            bCords->updatePosition(BcurrentPosition.xH+BlegR->heightAt(),BcurrentPosition.xFB+BlegR->fbAt(),BcurrentPosition.xLR+BlegR->lrAt());
            cCords->updatePosition(CcurrentPosition.xH+ClegR->heightAt(),CcurrentPosition.xFB+ClegR->fbAt(),CcurrentPosition.xLR+ClegR->lrAt());
            dCords->updatePosition(DcurrentPosition.xH+DlegR->heightAt(),DcurrentPosition.xFB+DlegR->fbAt(),DcurrentPosition.xLR+DlegR->lrAt());
        }
        setLegPositions();
    }
};