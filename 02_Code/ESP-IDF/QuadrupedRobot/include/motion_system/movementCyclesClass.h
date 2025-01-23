

#ifndef movementCyclesClass
#define movementCyclesClass
#include "singleCycleClass.h"
class movementCycles{
    public:
    int currentCycleIndex;
    singleCycle* cycle;
    float totalTime;
    int cycleCount;
    bool absolutePositioning;
    bool direction;
    movementCycles(int cycleCount, bool absolutePositioning, bool direction, float totalTime, singleCycle* cycle)
        : currentCycleIndex(0), cycle(cycle), totalTime(totalTime), cycleCount(cycleCount), absolutePositioning(absolutePositioning), direction(direction) {
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
#endif