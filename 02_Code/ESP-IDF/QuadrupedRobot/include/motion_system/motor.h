

#ifndef __motorClass__
#define __motorClass__
#include "../drivers/pca9685.h"
#define servo_max 2500
#define servo_min 700
class motor{
  private:
    pca9685 *pwm;
    uint16_t Hlimit;
    uint16_t Llimit;
    bool direction;
    float* offset;
    uint8_t motorC;
    float Degree;
  public:
  motor(pca9685* pwmV, uint8_t motorV, uint16_t LlimitV, uint16_t HlimitV, bool directionV, float* offsetV);
  void set_degree(float nDegree);
  uint8_t getMotor();
};
#endif