#include "../../include/motion_system/ramp_leg.h"

rampLeg::rampLeg(uint8_t mhip) : hip(mhip), xH(0), xFB(0), xLR(0)
{
    hip = mhip;
    xH.set_target(0, 0);
    xFB.set_target(0, 0);
    xLR.set_target(0, 0);
}


uint8_t rampLeg::getMotor()
{
    return hip;
}


bool rampLeg::allDone()
{
    if (xH.is_complete() && xFB.is_complete() && xLR.is_complete())
    {
        return true;
    }
    else
    {
        return false;
    }
}


void rampLeg::setPositions(float VH, float VLR, float VFB, float timee)
{
    xH.set_target(VH, timee);
    xFB.set_target(VFB, timee);
    xLR.set_target(VLR, timee);
}


void rampLeg::reset()
{
    xH.set_target(0, 0);
    xFB.set_target(0, 0);
    xLR.set_target(0, 0);
}


void rampLeg::update()
{
    xH.get_position();
    xLR.get_position();
    xFB.get_position();
}



void rampLeg::hGo(float position, float timee)
{
    xH.set_target(position, timee);
}


void rampLeg::fbGo(float position, float timee)
{
    xFB.set_target(position, timee);
}


void rampLeg::lrGo(float position, float timee)
{
    xLR.set_target(position, timee);
}


float rampLeg::heightAt()
{
    return xH.get_position();
}


float rampLeg::fbAt()
{
    return xFB.get_position();
}


float rampLeg::lrAt()
{
    return xLR.get_position();
}


bool rampLeg::isGrounded()
{
    if (xFB.get_position() == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}