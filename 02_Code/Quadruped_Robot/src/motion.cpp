#include "externFunctions.h"
#include <Arduino.h>


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
};

class movementCycles{
    int currentCycleIndex;
    public:
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
        if(currentCycleIndex > cycleCount){
            currentCycleIndex = 0;
        }else{
            currentCycleIndex++;
        }
    }
    float *getPositons(){
        return cycle[currentCycleIndex].legPositions;
    }
};

float cycleTime = 50;
float moveBackDistance = -40;
float moveUpDistance =-40;
//AxH,AxLR,AxFB,BxH,BxLR,BxFB,CxH,CxLR,CxFB,DxH,DxLR,DxFB
singleCycle walking0(0,0,moveBackDistance/3, // moving back
                    moveUpDistance,0,2*moveBackDistance/3, //returning to 0
                    moveUpDistance,0,2*moveBackDistance/3, //returning to 0
                    0,0,moveBackDistance/3); // moving back


singleCycle walking1(0,0,2*moveBackDistance/3, // moving back
                    moveUpDistance,0,moveBackDistance/3, //returning to 0
                    moveUpDistance,0,moveBackDistance/3,+//returning to 0
                    0,0,2*moveBackDistance/3); // moving back


singleCycle walking2(0,0,moveBackDistance, //moved back
                    0,0,0, //back at 0
                    0,0,0, //back at 0
                    0,0,moveBackDistance); //moved back


singleCycle walking3(moveUpDistance,0,2*moveBackDistance/3, //returning to 0
                    0,0,moveBackDistance/3, //moving back
                    0,0,moveBackDistance/3, //moving back
                    moveUpDistance,0,2*moveBackDistance/3); //returning to 0

singleCycle walking4(moveUpDistance,0,moveBackDistance/3, //returning to 0
                    0,0,2*moveBackDistance/3, // moving back
                    0,0,2*moveBackDistance/3, // moving back
                    moveUpDistance,0,moveBackDistance/3); //returning to 0

singleCycle walking5(0,0,0, //back at 0
                    0,0,moveBackDistance, //moved back
                    0,0,moveBackDistance, //moved back
                    0,0,0); //back at 0


singleCycle walking[] = {walking0,walking1,walking2,walking3,walking4,walking5};
movementCycles walkForward(6, false,true,3,walking);


class cycleControl{
    movementCycles &activeCycle;

    rampLeg &AlegR;
    rampLeg &BlegR;
    rampLeg &ClegR;
    rampLeg &DlegR;

    kinematics &AlegK;
    kinematics &BlegK;
    kinematics &ClegK;
    kinematics &DlegK;

    Cords &aCords;
    Cords &bCords;
    Cords &cCords;
    Cords &dCords;

    Cords AcurrentPosition;
    Cords BcurrentPosition;
    Cords CcurrentPosition;
    Cords DcurrentPosition;

    int currentStage = 0;
    cycleControl(movementCycles &activeCycle, 
    rampLeg &AlegR, rampLeg &BlegR, rampLeg &ClegR, rampLeg &DlegR, 
    kinematics &AlegK,kinematics &BlegK, kinematics &ClegK, kinematics &DlegK, 
    Cords &aCords, Cords &bCords, Cords &cCords, Cords &dCords) : 
    activeCycle(activeCycle),
    AlegR(AlegR), BlegR(BlegR), ClegR(ClegR), DlegR(DlegR),
    AlegK(AlegK), BlegK(BlegK), ClegK(ClegK), DlegK(DlegK),
    aCords(aCords), bCords(bCords), cCords(cCords), dCords(dCords) {}

    void updateRampPositions(){
        //update ramps
        AlegR.update();
        BlegR.update();
        ClegR.update();
        DlegR.update();
    }

    void checkCycle(){
        if(AlegR.allDone() && BlegR.allDone() && ClegR.allDone() && DlegR.allDone()){
            //go to next cycle if done
            activeCycle.nextCycle();
            float cycleTime = activeCycle.totalTime/activeCycle.cycleCount;
            //set new positons and time
            AlegR.hGo(activeCycle.cycle->legPositions[0],cycleTime);
            AlegR.lrGo(activeCycle.cycle->legPositions[1],cycleTime);
            AlegR.fbGo(activeCycle.cycle->legPositions[2],cycleTime);

            BlegR.hGo(activeCycle.cycle->legPositions[3],cycleTime);
            BlegR.lrGo(activeCycle.cycle->legPositions[4],cycleTime);
            BlegR.fbGo(activeCycle.cycle->legPositions[5],cycleTime);

            ClegR.hGo(activeCycle.cycle->legPositions[6],cycleTime);
            ClegR.lrGo(activeCycle.cycle->legPositions[7],cycleTime);
            ClegR.fbGo(activeCycle.cycle->legPositions[8],cycleTime);

            DlegR.hGo(activeCycle.cycle->legPositions[9],cycleTime);
            DlegR.lrGo(activeCycle.cycle->legPositions[10],cycleTime);
            DlegR.fbGo(activeCycle.cycle->legPositions[11],cycleTime);
        }
    }

    void setLegPositions(){
        AlegK.mainKinematics(aCords);
        BlegK.mainKinematics(bCords);
        ClegK.mainKinematics(cCords);
        DlegK.mainKinematics(dCords);
    }

    void continueCycle(){
        checkCycle();

        updateRampPositions();

        if(activeCycle.absolutePositioning){
            aCords.updatePosition(AlegR.heightAt(),AlegR.fbAt(),AlegR.lrAt());
            bCords.updatePosition(BlegR.heightAt(),BlegR.fbAt(),BlegR.lrAt());
            cCords.updatePosition(ClegR.heightAt(),ClegR.fbAt(),ClegR.lrAt());
            dCords.updatePosition(DlegR.heightAt(),DlegR.fbAt(),DlegR.lrAt());

        }else{
            aCords.updatePosition(AcurrentPosition.xH+AlegR.heightAt(),AcurrentPosition.xFB+AlegR.fbAt(),AcurrentPosition.xLR+AlegR.lrAt());
            bCords.updatePosition(BcurrentPosition.xH+BlegR.heightAt(),BcurrentPosition.xFB+BlegR.fbAt(),BcurrentPosition.xLR+BlegR.lrAt());
            cCords.updatePosition(CcurrentPosition.xH+ClegR.heightAt(),CcurrentPosition.xFB+ClegR.fbAt(),CcurrentPosition.xLR+ClegR.lrAt());
            dCords.updatePosition(DcurrentPosition.xH+DlegR.heightAt(),DcurrentPosition.xFB+DlegR.fbAt(),DcurrentPosition.xLR+DlegR.lrAt());


        }
        setLegPositions();
    }
};