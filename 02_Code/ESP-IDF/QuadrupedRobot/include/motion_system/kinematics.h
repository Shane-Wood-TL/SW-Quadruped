

#ifndef kinematicsClass
#define kinematicsClass

#include "../all_includes.h"
#include "leg.h"
#include "structures.h"
#include "../hardware_setup/robot_constants.h"
#include "../math_functions.h"


class kinematics{
  private:
  leg *legC;

  public:
  kinematics(leg* legV);
  void mainKinematics(position current_position);
};
#endif