

#ifndef rampLegClass
#define rampLegClass
#include "../interpolators/interpolation.h"
#include <cstdint>
class rampLeg{
  private:
    uint8_t hip;
    interpolation  xH;
    interpolation xFB;
    interpolation xLR;
  public:
    rampLeg(uint8_t mhip);
    uint8_t getMotor();
    bool allDone();
    void setPositions(float VH, float VLR, float VFB, float timee);
    void reset();
    void update();
    void hGo(float position, float timee);
    void fbGo(float position, float timee);
    void lrGo(float position, float timee);
    float heightAt();
    float fbAt();
    float lrAt();
    bool isGrounded();
};

#endif