#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <externFunctions.h>

//motor drivers defined in main
extern Adafruit_PWMServoDriver pwm;
extern Adafruit_PWMServoDriver pwm1;

//servo motor pwm limits
#define USMIN 771  // min value given from arduino lib
#define USMAX 2193 // max value given from arduino lib


//dimensions of the bot
const int yHalfDis = 85;
const int zHalfDis = 190;
// constant distances
const float aLength = 95; // upper leg length more red
const float bLength = 95; // lower leg length more purple
const float Ldis = 45;

// motor definitions
const int aHip = 3;   //
const int aKnee = 4;  //
const int aAnkle = 5; //

const int bHip = 2;   //
const int bKnee = 1;  //
const int bAnkle = 0; //

const int cHip = 2;   //
const int cKnee = 1;  //
const int cAnkle = 0; //

const int dHip = 3;    //
const int dKnee = 4;  //
const int dAnkle = 5; //


// DO NOT CHANGE, UNLESS MOTOR SWAP
// B-----A
// |     |
// |     |
// |     |
// |     |
// |     |
// D-----C
//  front

const float aHipOffset = 0; 
const float aKneeOffset =40; 
const float aAnkleOffset = -20;

const float bHipOffset = 0;
const float bKneeOffset = 20;
const float bAnkleOffset = 0;

const float cHipOffset = 14;
const float cKneeOffset = -10;
const float cAnkleOffset = -35;

const float dHipOffset = -4;
const float dKneeOffset = 85;
const float dAnkleOffset = 35;


//code that creates the angles for the motors based on position and angles
void mainKinematics(float xH, float xFB, float xLR, int hipMotor, float xRot, float yRot, float zRot)
{
  float innerAngleKneeA, kneeAngle, outerAngleKneeB;
  float zMultiplier = 1;
  float yMultiplier = 1;

  float yRotR = decrad((yRot));
  float yAddition = yMultiplier * yHalfDis * tan(yRotR);

  if ((hipMotor == 0 || hipMotor == 6))
  {
    if (yRot < 0)
    {
      xH += yAddition;
    }
    else
    {
      xH+= yAddition;
    }
  }
  else if ((hipMotor == 3 || hipMotor == 9))
  {
    if (yRot < 0)
    {
      xH -= yAddition;
    }
    else
    {
      xH -= yAddition;
    }
  }

  float zRotR = decrad((zRot));
  float zAddition = zHalfDis * tan(zRotR);
  if ((hipMotor == 0 || hipMotor ==3))
  {
    if (zRot < 0)
    {
      xFB += zAddition * zMultiplier;
    }
    else
    {
      xFB += zAddition * zMultiplier;
    }
  }
  else if ((hipMotor ==  6 || hipMotor == 9))
  {
    if (zRot < 0)
    {
      xFB -= zAddition * zMultiplier;
    }
    else
    {
      xFB -= zAddition * zMultiplier;
    }
  }

//hard coded saftey values
if (true) {
  if(xH >= 190){
    xH = 190;
  }else if (xH<= 20){
    xH = 20;
  }

  if (xFB >= 50){
    xFB = 50;
  }else if(xFB <=-70){
    xFB = -70;
  }

  if(xLR >= 50){
    xLR = 50;
  }else if(xLR <= -25){
    xLR = -25;
  }
}
  // hip angle
  float innerLegLength = pytherm(xH, Ldis + xLR);
  // now must solve for 1st purple angle
  // using cos, a/h
  // now for the second triangle of the first part
  float modLegL = pythermhypt(Ldis, innerLegLength); // most outer triangle
  float innerAngleA = acos(xH / innerLegLength); // adj / hyp
  innerAngleA = raddec(innerAngleA);
  // now must solve for the 2nd purple angle
  float innerAngleB = acos(Ldis / innerLegLength); // adjacent and hyp cah soh
  innerAngleB = raddec(innerAngleB);
  // everything up to this point works still
  // write the 2 values the the hip motor at the end now
  // time for stage 2
  float modLegLL = pytherm(xFB, modLegL);
  // solve for this angle aswell

  if (xFB <= 0) //BACKWARDS
  {
    xFB = abs(xFB);
    float innerAngleKneeA = acos(modLegL / modLegLL); // cah
    innerAngleKneeA = raddec(innerAngleKneeA);
    // solve for the other triangle
    //  LAW OF COSINES NEEDED, all sides found
    float outerAngleKneeB = loc(aLength, modLegLL, bLength);
    // now finally the last value can be passed to the ankle servo
    // using loc to find the angle for the ankle
    float kneeAngle = loc(aLength, bLength, modLegLL);

    
    if (hipMotor == 0)
    {
      pwm.writeMicroseconds(aHip, map((innerAngleA + innerAngleB + aHipOffset), 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(aKnee, map((outerAngleKneeB + innerAngleKneeA + aKneeOffset), 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(aAnkle, (map((kneeAngle + aAnkleOffset), 0, 180, USMIN, USMAX)));

    }
    else if (hipMotor == 3)
    {
      pwm.writeMicroseconds(bHip, map(180-(innerAngleA + innerAngleB + bHipOffset), 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(bKnee, map(180-(outerAngleKneeB + innerAngleKneeA + bKneeOffset), 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(bAnkle, map(180-(kneeAngle + bAnkleOffset), 0, 180, USMIN, USMAX));

    }
    else if (hipMotor == 6)
    {
      pwm1.writeMicroseconds(cHip, map(180-(innerAngleA + innerAngleB + cHipOffset), 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(cKnee, map((outerAngleKneeB + innerAngleKneeA) + cKneeOffset, 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(cAnkle, map((kneeAngle + cAnkleOffset), 0, 180, USMIN, USMAX));

    }
    else if (hipMotor == 9)
    {
      pwm1.writeMicroseconds(dHip, map((innerAngleA + innerAngleB + dHipOffset), 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(dKnee, map(180-(outerAngleKneeB + innerAngleKneeA + dKneeOffset), 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(dAnkle, map(180-(kneeAngle + dAnkleOffset), 0, 180, USMIN, USMAX));

    }
  }
  else //FORWARDS
  {
    float innerAngleKneeA = acos(modLegL / modLegLL); // cah
    float innerAngleKneeB = loc(aLength, modLegLL, bLength);
    

    // need to write 90 + (b-a)
    float kneeAngle = loc(aLength, bLength, modLegLL);
    if (hipMotor == 0)
    {
      // offsets for a
      pwm.writeMicroseconds(aHip, map((innerAngleA + innerAngleB + aHipOffset), 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(aKnee, map(90+(innerAngleKneeA+ aKneeOffset-innerAngleB), 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(aAnkle, map((kneeAngle +aAnkleOffset), 0, 180, USMIN, USMAX));

      // Serial.println(innerAngleKneeA+outerAngleKneeB);
    }
    else if (hipMotor == 3)
    {
      // offsets for b
      pwm.writeMicroseconds(bHip, map(180-(innerAngleA + innerAngleB+bHipOffset), 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(bKnee, map(90-(innerAngleKneeA+bKneeOffset-innerAngleB), 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(bAnkle, map(180-(kneeAngle + bAnkleOffset), 0, 180, USMIN, USMAX));
    }
    else if (hipMotor == 6)
    {
      // offsets for c
      pwm1.writeMicroseconds(cHip, map(180-(innerAngleA + innerAngleB+cHipOffset), 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(cKnee, map(90+(innerAngleKneeA+cKneeOffset-innerAngleB), 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(cAnkle, map((kneeAngle + cAnkleOffset), 0, 180, USMIN, USMAX));
    }
    else if (hipMotor == 9)
    {
      // offsets for d
      pwm1.writeMicroseconds(dHip, map((innerAngleA + innerAngleB +dHipOffset), 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(dKnee, map(90-(innerAngleKneeA + dKneeOffset-innerAngleB), 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(dAnkle, map(180-(kneeAngle + dAnkleOffset), 0, 180, USMIN, USMAX));
    }
  }
}

//used to ensure that new motors are in a close position to what is "correct"
void all_90s(){
      pwm.writeMicroseconds(aHip, map(90, 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(aKnee, map(90, 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(aAnkle, map(90, 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(bHip, map(90, 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(bKnee, map(90, 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(bAnkle, map(90, 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(cHip, map(90, 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(cKnee, map(90, 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(cAnkle, map(90, 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(dHip, map(90, 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(dKnee, map(90, 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(dAnkle, map(90, 0, 180, USMIN, USMAX));
}