#include <Kinematics/motorClass.h>

#ifndef legClass
#define legClass
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
      hip->setDegree(hipV);
      knee->setDegree(kneeV);
      ankle->setDegree(ankleV);
    }
    char getLegName(){
      return name;
    }
};
#endif