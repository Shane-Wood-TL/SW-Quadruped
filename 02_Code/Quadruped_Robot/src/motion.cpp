// #include "externFunctions.h"
// #include <Arduino.h>


// struct singleCycle {
//     float legPositions[12];

//     singleCycle(float AxH, float AxLR, float AxFB,
//                 float BxH, float BxLR, float BxFB,
//                 float CxH, float CxLR, float CxFB,
//                 float DxH, float DxLR, float DxFB) {
        
//         legPositions[0] = AxH;
//         legPositions[1] = AxLR;
//         legPositions[2] = AxFB;

//         legPositions[3] = BxH;
//         legPositions[4] = BxLR;
//         legPositions[5] = BxFB;

//         legPositions[6] = CxH;
//         legPositions[7] = CxLR;
//         legPositions[8] = CxFB;

//         legPositions[9] = DxH;
//         legPositions[10] = DxLR;
//         legPositions[11] = DxFB;
//     }
// };

// class movementCycles{
//     int cycleCount;
//     bool absolutePositioning;
//     bool direction;
//     float totalTime;
//     singleCycle* cycle;

//     public:
//     movementCycles(int cycleCount, bool absolutePositioning, bool direction, float totalTime, singleCycle* cycle)
//         : cycleCount(cycleCount), absolutePositioning(absolutePositioning), direction(direction), totalTime(totalTime), cycle(cycle) {}
// };


// //AxH,AxLR,AxFB,BxH,BxLR,BxFB,CxH,CxLR,CxFB,DxH,DxLR,DxFB
// singleCycle walking0(-50,0,0,0,0,0,-50,0,0,0,0,0);
// singleCycle walking1(-50,0,0,0,0,0,-50,0,0,0,0,0);
// singleCycle walking2(-50,0,0,0,0,0,-50,0,0,0,0,0);
// singleCycle walking3(-50,0,0,0,0,0,-50,0,0,0,0,0);
// singleCycle walking4(0,0,0,-50,0,0,0,0,0,-50,0,0);
// singleCycle walking5(0,0,0,-50,0,0,0,0,0,-50,0,0);
// singleCycle walking6(0,0,0,-50,0,0,0,0,0,-50,0,0);
// singleCycle walking7(0,0,0,-50,0,0,0,0,0,-50,0,0);

// singleCycle walking[] = {walking0,walking1,walking2,walking3,walking4,walking5,walking6};
// movementCycles walkForward(7, false,true,3,walking);