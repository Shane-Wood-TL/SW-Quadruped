#ifndef legClass
#define legClass
#include "motor.h"
class leg{
  motor* hip;
  motor* knee;
  motor* ankle;
  char name;
  public:
    leg(motor* hipV, motor* kneeV, motor* ankleV, const char nameV);
    void setAngles(float hipV, float kneeV, float ankleV);
    char getLegName();
};
#endif