#include <Arduino.h>

#ifndef PayloadStructF
#define PayloadStructF
struct PayloadStruct {
  uint8_t eStop; //sw2
  uint8_t state;
  uint8_t gyro;
  uint8_t PID;
  int16_t j1_x;
  int16_t j1_y;
  int16_t j1_z;
  int16_t j2_x;
  int16_t j2_y;
  int16_t j2_z;
  uint8_t bt0_;
  uint8_t bt1_;
  uint8_t bt2_;
};
#endif

