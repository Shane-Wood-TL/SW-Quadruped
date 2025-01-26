#ifndef __ramped_kinematics__
#define __ramped_kinematics__
#include "ramp_leg.h"
#include "kinematics.h"
#include "positioning/cycle.h"
class ramped_kinematics{
    kinematics *kinematic_driver;
    rampLeg *ramped_driver;
    public:
    ramped_kinematics(kinematics *kinematic_driver, rampLeg *ramped_driver);
    bool all_done();
    void update();
    stance get_current_stance();
    void set_stance(single_leg new_stance);
    void set_interpolated_stance(single_leg new_stance, float time_to_stance);
    void stop();
};
#endif