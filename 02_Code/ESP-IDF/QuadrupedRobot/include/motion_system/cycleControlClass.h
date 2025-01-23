

#ifndef cycleControlClass
#define cycleControlClass
#include "cordsClass.h"
#include "rampLegClass.h"
#include "kinematicsClass.h"
#include "movementCyclesClass.h"


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

#endif