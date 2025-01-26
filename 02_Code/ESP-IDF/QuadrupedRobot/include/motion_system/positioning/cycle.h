#ifndef __cycle__
#define __cycle__

#include "stance.h"
#include <cstdint>

class cycle{
    float total_time_for_cycle;
    uint8_t cycle_length;
    stance **stances;
    uint8_t current_stance_index;
    bool direction;
    public:
        cycle(uint8_t cycle_length, stance *stances[], bool direction);
        stance get_current_stance();
        void next_stance();
        float get_stance_time();
};

#endif