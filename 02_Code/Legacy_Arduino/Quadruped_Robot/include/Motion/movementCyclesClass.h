#include <motion/singleCycleClass.h>

#ifndef movementCyclesClass
#define movementCyclesClass
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
#endif