#include <allIncludes.h>

extern singleCycle activeOffsets;


void applyBaseOffsets(){
    activeOffsets.legPositions[0] = A_HIP_OFFSET;
    activeOffsets.legPositions[1] = A_KNEE_OFFSET;
    activeOffsets.legPositions[2] = A_ANKLE_OFFSET;

    activeOffsets.legPositions[3] = B_HIP_OFFSET;
    activeOffsets.legPositions[4] = B_KNEE_OFFSET;
    activeOffsets.legPositions[5] = B_ANKLE_OFFSET;

    activeOffsets.legPositions[6] = C_HIP_OFFSET;
    activeOffsets.legPositions[7] = C_KNEE_OFFSET;
    activeOffsets.legPositions[8] = C_ANKLE_OFFSET;

    activeOffsets.legPositions[9] = D_HIP_OFFSET;
    activeOffsets.legPositions[10] = D_KNEE_OFFSET;
    activeOffsets.legPositions[11] = D_ANKLE_OFFSET;
}