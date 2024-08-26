void applyBaseOffsets();

#define A_HIP_OFFSET -6
#define A_KNEE_OFFSET 5
#define A_ANKLE_OFFSET -23

#define B_HIP_OFFSET 3
#define B_KNEE_OFFSET 15
#define B_ANKLE_OFFSET -20

#define C_HIP_OFFSET 5
#define C_KNEE_OFFSET 5
#define C_ANKLE_OFFSET -15

#define D_HIP_OFFSET -2
#define D_KNEE_OFFSET 10
#define D_ANKLE_OFFSET -20

class positions{
    public:
        float aHipV;
        float aKneeV;
        float aAnkleV;
        float bHipV;
        float bKneeV;
        float bAnkleV;
        float cHipV;
        float cKneeV;
        float cAnkleV;
        float dHipV;
        float dKneeV;
        float dAnkleV;
        positions(){
            aHipV = 0;
            aKneeV = 0;
            aAnkleV = 0;
            bHipV = 0;
            bKneeV = 0;
            bAnkleV = 0;
            cHipV = 0;
            cKneeV = 0;
            cAnkleV = 0;
            dHipV = 0;
            dKneeV = 0;
            dAnkleV = 0;
        }
        void resetPositions(){
            aHipV = 0;
            aKneeV = 0;
            aAnkleV = 0;
            bHipV = 0;
            bKneeV = 0;
            bAnkleV = 0;
            cHipV = 0;
            cKneeV = 0;
            cAnkleV = 0;
            dHipV = 0;
            dKneeV = 0;
            dAnkleV = 0;
        }
        void setOffsets(positions newOffsets){
            aHipV = newOffsets.aHipV;
            aKneeV = newOffsets.aKneeV;
            aAnkleV = newOffsets.aAnkleV;

            bHipV = newOffsets.bHipV;
            bKneeV = newOffsets.bKneeV;
            bAnkleV = newOffsets.bAnkleV;

            cHipV = newOffsets.cHipV;
            cKneeV = newOffsets.cKneeV;
            cAnkleV = newOffsets.cAnkleV;

            dHipV = newOffsets.dHipV;
            dKneeV = newOffsets.dKneeV;
            dAnkleV = newOffsets.dAnkleV;
        }

};

extern positions activeOffsets;


void applyBaseOffsets(){
    activeOffsets.aHipV = A_HIP_OFFSET;
    activeOffsets.aKneeV = A_KNEE_OFFSET;
    activeOffsets.aAnkleV = A_ANKLE_OFFSET;

    activeOffsets.bHipV = B_HIP_OFFSET;
    activeOffsets.bKneeV = B_KNEE_OFFSET;
    activeOffsets.bAnkleV = B_ANKLE_OFFSET;

    activeOffsets.cHipV = C_HIP_OFFSET;
    activeOffsets.cKneeV = C_KNEE_OFFSET;
    activeOffsets.cAnkleV = C_ANKLE_OFFSET;

    activeOffsets.dHipV = D_HIP_OFFSET;
    activeOffsets.dKneeV = D_KNEE_OFFSET;
    activeOffsets.dAnkleV = D_ANKLE_OFFSET;
}
