#ifndef singleCycleClass
#define singleCycleClass
class singleCycle {
    public:
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
    singleCycle() {
        
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
       legPositions[0] = newOffsets.legPositions[0];
        legPositions[1] = newOffsets.legPositions[1];
        legPositions[2] = newOffsets.legPositions[2];

        legPositions[3] = newOffsets.legPositions[3];
        legPositions[4] = newOffsets.legPositions[4];
        legPositions[5] = newOffsets.legPositions[5];

        legPositions[6] = newOffsets.legPositions[6];
        legPositions[7] = newOffsets.legPositions[7];
        legPositions[8] = newOffsets.legPositions[8];

        legPositions[9] = newOffsets.legPositions[9];
        legPositions[10] = newOffsets.legPositions[10];
        legPositions[11] = newOffsets.legPositions[11];
    }
};

#endif