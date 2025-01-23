#include <Adafruit_PWMServoDriver.h>
#include <robotConstants.h>

#ifndef motorClass
#define motorClass
class motor{
  private:
    Adafruit_PWMServoDriver* pwm;
    float Hlimit;
    float Llimit;
    bool direction;
    float* offset;
    int motorC;
  public:
  motor(Adafruit_PWMServoDriver* pwmV, int motorV, float LlimitV, float HlimitV, bool directionV, float* offsetV){
    pwm = pwmV;
    motorC = motorV;
    Llimit = LlimitV;
    Hlimit = HlimitV;
    direction = directionV;
    offset = offsetV;
  }
    float Degree;
  void setDegree(float nDegree){
    if(nDegree+*offset > Hlimit){
      nDegree = Hlimit-*offset;
    }else if(nDegree+*offset < Llimit){
      nDegree = Llimit-*offset; 
    }else{
        nDegree = nDegree + *offset;
    }
    if(direction){
        pwm->writeMicroseconds(motorC, (map(nDegree, 0, 180, USMIN, USMAX)));
    }else{
        pwm->writeMicroseconds(motorC, (map(nDegree, 180, 0, USMIN, USMAX)));
    }
  }
  int getMotor(){
    return motorC;
  }
};
#endif