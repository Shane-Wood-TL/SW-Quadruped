#ifndef __stance__
#define __stance__

#include "structures.h"

typedef struct{
    single_leg *A_positions;
    single_leg *B_positions;
    single_leg *C_positions;
    single_leg *D_positions;
    float time_for_stance =0;
} stance;
#endif