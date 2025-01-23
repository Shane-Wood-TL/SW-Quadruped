#include "../../include/motion_system/kinematics.h"

kinematics::kinematics(leg *legV)
{
    legC = legV;
}

void kinematics::mainKinematics(single_leg position)
{
    // code to convert angles to change in positions
    float modZd;
    float modYd;
    float xHz;
    float xHy;
    float xFBz;
    float xLRy;

    xHz = zHalfDis * sin(decrad(position.rot_z));
    modZd = zHalfDis * cos(decrad(position.rot_z));

    if (legC->getLegName() == 'C' || legC->getLegName() == 'D')
    {
        position.xH -= xHz;
        position.xFB -= modZd - zHalfDis;
    }
    else
    {
        position.xH += xHz;
        position.xFB += modZd - zHalfDis;
    }

    xHy = yHalfDis * sin(decrad(position.rot_y));
    modYd = yHalfDis * cos(decrad(position.rot_y));

    if (legC->getLegName() == 'B' || legC->getLegName() == 'D')
    {
        position.xH -= xHy;
        position.xLR -= modYd - yHalfDis;
    }
    else
    {
        position.xH += xHy;
        position.xLR += modYd - yHalfDis;
    }

    if (position.xH > 134)
    {
        position.xH = 134;
    }

    float innerLegLength = pytherm(position.xH, Ldis + position.xLR);
    float modLegL = pythermhypt(Ldis, innerLegLength);
    float innerAngleA = acos(position.xH / innerLegLength);
    innerAngleA = raddec(innerAngleA);
    float innerAngleB = acos(Ldis / innerLegLength);
    innerAngleB = raddec(innerAngleB);
    float modLegLL = pytherm(position.xFB, modLegL);
    float innerAngleKneeA = acos(modLegL / modLegLL);
    innerAngleKneeA = raddec(innerAngleKneeA);
    float outerAngleKneeB = loc(aLength, modLegLL, bLength);
    float kneeAngle = loc(aLength, bLength, modLegLL);
    if (position.xFB <= 0)
    {
        legC->setAngles(innerAngleA + innerAngleB, 90 - (outerAngleKneeB + innerAngleKneeA), kneeAngle);
    }
    else
    {
        legC->setAngles(innerAngleA + innerAngleB, 90 - (outerAngleKneeB - innerAngleKneeA), kneeAngle);
    }
}
