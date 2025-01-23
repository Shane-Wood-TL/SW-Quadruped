#include "../../include/motion_system/leg.h"

leg::leg(motor *hipV, motor *kneeV, motor *ankleV, const char nameV)
{
    hip = hipV;
    knee = kneeV;
    ankle = ankleV;
    name = nameV;
}

void leg::setAngles(float hipV, float kneeV, float ankleV)
{
    hip->set_degree(hipV);
    knee->set_degree(kneeV);
    ankle->set_degree(ankleV);
}
char leg::getLegName()
{
    return name;
}