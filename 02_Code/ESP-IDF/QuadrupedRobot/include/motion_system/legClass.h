

#ifndef legClass
#define legClass
#include "motor.h"
class leg{
  motor* hip;
  motor* knee;
  motor* ankle;
  char name;
  public:
    leg(motor* hipV, motor* kneeV, motor* ankleV, const char nameV){
      hip = hipV;
      knee = kneeV;
      ankle = ankleV;
      name = nameV;
    }

    void setAngles(float hipV, float kneeV, float ankleV){
      hip->set_degree(hipV);
      knee->set_degree(kneeV);
      ankle->set_degree(ankleV);
    }
    char getLegName(){
      return name;
    }
};
#endif