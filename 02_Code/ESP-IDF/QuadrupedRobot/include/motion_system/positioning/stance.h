#ifndef __stance__
#define __stance__

#include "../structures.h"

typedef struct{
    position *A_positions;
    position *B_positions;
    position *C_positions;
    position *D_positions;
    float time_for_stance =0;
} stance;
#endif