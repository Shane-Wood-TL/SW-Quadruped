#ifndef __robot_movement__
#define __robot_movement__

#include "ramped_kinematics.h"


class robot_movement{
    private:
        ramped_kinematics *a_leg;
        ramped_kinematics *b_leg;
        ramped_kinematics *c_leg;
        ramped_kinematics *d_leg;
        cycle *current_cycle;
        
    public:
        void update();
        robot_movement(ramped_kinematics *a_leg, ramped_kinematics *b_leg, ramped_kinematics *c_leg, ramped_kinematics *d_leg, cycle *starting_cycle);
        void set_cycle(cycle *current_cycle);
        bool all_done();
};

#endif