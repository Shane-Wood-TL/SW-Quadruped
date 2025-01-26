#include "../../../include/motion_system/positioning/cycle.h"
#include "../../../include/all_includes.h"

cycle::cycle(uint8_t cycle_length, stance **stances, bool direction){
    current_stance_index = 0;
    this->cycle_length = cycle_length;
    this->stances = stances;
    this->direction = direction;
    for(uint8_t i =0; i< cycle_length; i++){
        total_time_for_cycle += (stances[i])->time_for_stance;
    }
    
}

stance cycle::get_current_stance(){
    printf("Stance getting: %d \n", current_stance_index);
    return (*stances[current_stance_index]);
}

void cycle::next_stance(){
    printf("Stance next: %d \n", current_stance_index);
    if (direction) {
        current_stance_index++;
        if (current_stance_index >= cycle_length) { 
            current_stance_index = 0;
        }
    } else { // Moving backward
        if (current_stance_index == 0) { 
            current_stance_index = cycle_length - 1;
        } else {
            current_stance_index--;
        }
    }
    printf("Stance new: %d \n", current_stance_index);
}

float cycle::get_stance_time(){
    return (*stances[current_stance_index]).time_for_stance;
}