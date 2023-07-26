#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Ramp.h>

extern Adafruit_PWMServoDriver pwm;
extern Adafruit_PWMServoDriver pwm1;

#define USMIN 544  // min value given from arduino lib
#define USMAX 2400 // max value given from arduino lib

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



const float aHipOffset = 7; // higher value = more out
const float aKneeOffset = 0; // higer value = cc
const float aAnkleOffset = 0; // higer value = smaller angle

const float bHipOffset = -5; // higher value = more in
const float bKneeOffset = -15;
const float bAnkleOffset = 0;

const float cHipOffset = 0; // higher value = more in
const float cKneeOffset = -10;
const float cAnkleOffset = -25;

const float dHipOffset = -2;     // higher value = more out
const float dKneeOffset = 0;   // higer value = cc
const float dAnkleOffset = -25; // higer value = larger angle


float pytherm(float sidea, float sideb);     // returns hypotenuse c
float raddec(float rad);                     // radians to degress
float loc(float a, float b, float c);        // law of cosines, returns angle c in degress
float pythermhypt(float sidea, float sidec); // returns side b
float decrad(float deg);                     // degrees to radians


extern ramp aHeight;
extern ramp bHeight;
extern ramp cHeight;
extern ramp dHeight;

extern ramp aFB;
extern ramp bFB;
extern ramp cFB;
extern ramp dFB;

extern ramp aLR;
extern ramp bLR;
extern ramp cLR;
extern ramp dLR;



// given height, forward/back, left/right, hip motor num, knee motor num, ankle motor num, bool if on the other side.
void mainKinematics(float xH, float xFB, float xLR, int hipMotor, float xRot, float yRot, float zRot)
{
  float innerAngleKneeA, kneeAngle, outerAngleKneeB; // cah
  float zMultiplier = 1;
  float yMultiplier = 1;
  // Serial.print("this is running");

  // angle code (no idea how s rotation works so for now it is going to be ignored)
  float yRotR = decrad((yRot));
  // float yRotR = decrad(abs(yRot));
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
  if ((hipMotor == 0 || hipMotor ==6))
  {
    if (zRot < 0)
    {
      xFB += zAddition * zMultiplier;
    }
    else
    {
      xFB -= zAddition * zMultiplier;
    }
  }
  else if ((hipMotor ==  3|| hipMotor == 9))
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

  /*
  this is intended to make the kinematic system into a single function that works for all legs
  it could be done with fewer inputs, however this is going to make it was easier for me
  */

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
    xLR = 25;
  }else if(xLR <= -25){
    xLR = -25;
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

  // Serial.println(innerAngleA);
  // Serial.println(innerAngleB);
  // everything up to this point works still

  // write the 2 values the the hip motor at the end now

  // time for stage 2
  float modLegLL = pytherm(xFB, modLegL);
  // solve for this angle aswell
  if (xFB <= 0)
  {
    xFB = abs(xFB);

    float innerAngleKneeA = acos(modLegL / modLegLL); // cah
    innerAngleKneeA = raddec(innerAngleKneeA);

    // solve for the other triangle
    //  LAW OF COSINES NEEDED, all sides found

    // float modLegLL= xH;
    float outerAngleKneeB = loc(aLength, modLegLL, bLength);
    // Serial.println(outerAngleKneeB);
    // now finally the last value can be passed to the ankle servo
    // using loc to find the angle for the ankle

    float kneeAngle = loc(aLength, bLength, modLegLL);

    // Serial.println(kneeAngle);
    // everything looks fine within the code, however i think something is still broken


    
    if (hipMotor == 0)
    {
      // offsets for a

      pwm.writeMicroseconds(aHip, map((innerAngleA + innerAngleB) + aHipOffset, 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(aKnee, map(180-(outerAngleKneeB + innerAngleKneeA) + aKneeOffset, 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(aAnkle, map((180-kneeAngle) + aAnkleOffset, 0, 180, USMIN, USMAX));
      //Serial.print(map((innerAngleA + innerAngleB) + aHipOffset, 0, 180, USMIN, USMAX));
      //Serial.print(",");
      //Serial.print(map((outerAngleKneeB + innerAngleKneeA) + aKneeOffset, 0, 180, USMIN, USMAX));
      //Serial.print(",");
      //Serial.print(map((kneeAngle) + aAnkleOffset, 0, 180, USMIN, USMAX));
      //Serial.print(",");
          // Serial.println(innerAngleKneeA+outerAngleKneeB);
    }
    else if (hipMotor == 3)
    {
      // offsets for b
      pwm.writeMicroseconds(bHip, map(((innerAngleA + innerAngleB)) + bHipOffset, 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(bKnee, map(((outerAngleKneeB + innerAngleKneeA)) + bKneeOffset, 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(bAnkle, map((kneeAngle) + bAnkleOffset, 0, 180, USMIN, USMAX));

      
     // Serial.print(map((180 - (innerAngleA + innerAngleB)) + bHipOffset, 0, 180, USMIN, USMAX));
      //Serial.print(",");
      //Serial.print(map((180 - (outerAngleKneeB + innerAngleKneeA)) + bKneeOffset, 0, 180, USMIN, USMAX));
      //Serial.print(",");
      //Serial.print(map((180 - kneeAngle) + bAnkleOffset, 0, 180, USMIN, USMAX));
      //Serial.print(",");
    }
    else if (hipMotor == 6)
    {
      // offsets for c
      pwm1.writeMicroseconds(cHip, map(((innerAngleA + innerAngleB)) + cHipOffset, 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(cKnee, map((180-(outerAngleKneeB + innerAngleKneeA)) + cKneeOffset, 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(cAnkle, map((180-(kneeAngle) + cAnkleOffset), 0, 180, USMIN, USMAX));

     // Serial.print(map((180 - (innerAngleA + innerAngleB)) + cHipOffset, 0, 180, USMIN, USMAX));
      //Serial.print(",");
     // Serial.print(map((180 - (outerAngleKneeB + innerAngleKneeA)) + cKneeOffset, 0, 180, USMIN, USMAX));
      //Serial.print(",");
     // Serial.print(map((180 - kneeAngle) + cAnkleOffset, 0, 180, USMIN, USMAX));
      //Serial.print(",");
    }
    else if (hipMotor == 9)
    {
      // offsets for d
      pwm1.writeMicroseconds(dHip, map(180-(innerAngleA + innerAngleB) + dHipOffset, 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(dKnee, map(((outerAngleKneeB + innerAngleKneeA)) + dKneeOffset, 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(dAnkle, map((kneeAngle) + dAnkleOffset, 0, 180, USMIN, USMAX));

     // Serial.print(map((innerAngleA + innerAngleB) + dHipOffset, 0, 180, USMIN, USMAX));
      //Serial.print(",");
      //Serial.print(map(((outerAngleKneeB + innerAngleKneeA)) + dKneeOffset, 0, 180, USMIN, USMAX));
      //Serial.print(",");
      //Serial.print(map((kneeAngle) + dAnkleOffset, 0, 180, USMIN, USMAX));
    }
  }
  else
  {
    float innerAngleKneeA = acos(modLegL / modLegLL); // cah
    float innerAngleKneeB = loc(aLength, modLegLL, bLength);
    float newKneeAngle = 90 + (innerAngleB - innerAngleA);

    // need to write 90 + (b-a)
    float kneeAngle = loc(aLength, bLength, modLegLL);
    if (hipMotor == 0)
    {
      // offsets for a
      pwm.writeMicroseconds(aHip, map((innerAngleA + innerAngleB) - aHipOffset, 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(aKnee, map(180 - newKneeAngle -aKneeOffset, 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(aAnkle, map((kneeAngle)-aAnkleOffset, 0, 180, USMIN, USMAX));

      // Serial.println(innerAngleKneeA+outerAngleKneeB);
    }
    else if (hipMotor == 3)
    {
      // offsets for b
      pwm.writeMicroseconds(bHip, map(((innerAngleA + innerAngleB))-bHipOffset, 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(bKnee, map(180-(newKneeAngle)-bKneeOffset, 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(bAnkle, map((180 - kneeAngle)-bAnkleOffset, 0, 180, USMIN, USMAX));
    }
    else if (hipMotor == 6)
    {
      // offsets for c
      pwm1.writeMicroseconds(cHip, map(180-((innerAngleA + innerAngleB))-cHipOffset, 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(cKnee, map((newKneeAngle)-cKneeOffset, 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(cAnkle, map(180-(kneeAngle)- cAnkleOffset, 0, 180, USMIN, USMAX));
    }
    else if (hipMotor == 9)
    {
      // offsets for d
      pwm1.writeMicroseconds(dHip, map(180-(innerAngleA + innerAngleB) -dHipOffset, 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(dKnee, map((180 - newKneeAngle) - dKneeOffset, 0, 180, USMIN, USMAX));
      pwm1.writeMicroseconds(dAnkle, map((180-kneeAngle)-dAnkleOffset, 0, 180, USMIN, USMAX));
    }
  }
}

void selfLevel()
{
  float xH = 100, xLR = 0, xFB = 0;
  float innerLegLength = pytherm(xH, Ldis + xLR);
  float modLegL = pythermhypt(Ldis, innerLegLength);
  float innerAngleA = acos(xH / innerLegLength);
  innerAngleA = raddec(innerAngleA);
  float innerAngleB = acos(Ldis / innerLegLength);
  innerAngleB = raddec(innerAngleB);
  float modLegLL = pytherm(xFB, modLegL);
  if (xFB <= 0)
  {
    xFB = abs(xFB);

    float innerAngleKneeA = acos(modLegL / modLegLL); // cah
    innerAngleKneeA = raddec(innerAngleKneeA);
    float outerAngleKneeB = loc(aLength, modLegLL, bLength);
    float kneeAngle = loc(aLength, bLength, modLegLL);

    float aHipOffset = -12; // higher value = more out
    float aKneeOffset = 12; // higer value = cc
    float aAnkleOffset = 0; // higer value = larger angle

    float bHipOffset = -8; // higher value = more in
    float bKneeOffset = -0;
    float bAnkleOffset = 15;

    float cHipOffset = -5; // higher value = more in
    float cKneeOffset = -0;
    float cAnkleOffset = 5;

    float dHipOffset = 0;     // higher value = more out
    float dKneeOffset = 12;   // higer value = cc
    float dAnkleOffset = -20; // higer value = larger angle
    // offsets for a

    pwm.writeMicroseconds(aHip, map((innerAngleA + innerAngleB) + aHipOffset, 0, 180, USMIN, USMAX));
    pwm.writeMicroseconds(aKnee, map((outerAngleKneeB + innerAngleKneeA) + aKneeOffset, 0, 180, USMIN, USMAX));
    pwm.writeMicroseconds(aAnkle, map((kneeAngle) + aAnkleOffset, 0, 180, USMIN, USMAX));

    // Serial.println(innerAngleKneeA+outerAngleKneeB);

    // offsets for b
    pwm.writeMicroseconds(bHip, map((180 - (innerAngleA + innerAngleB)) + bHipOffset, 0, 180, USMIN, USMAX));
    pwm.writeMicroseconds(bKnee, map((180 - (outerAngleKneeB + innerAngleKneeA)) + bKneeOffset, 0, 180, USMIN, USMAX));
    pwm.writeMicroseconds(bAnkle, map((180 - kneeAngle) + bAnkleOffset, 0, 180, USMIN, USMAX));

    // offsets for c
    pwm.writeMicroseconds(cHip, map((180 - (innerAngleA + innerAngleB)) + cHipOffset, 0, 180, USMIN, USMAX));
    pwm.writeMicroseconds(cKnee, map((180 - (outerAngleKneeB + innerAngleKneeA)) + cKneeOffset, 0, 180, USMIN, USMAX));
    pwm.writeMicroseconds(cAnkle, map((180 - kneeAngle) + cAnkleOffset, 0, 180, USMIN, USMAX));

    // offsets for d
    pwm.writeMicroseconds(dHip, map((innerAngleA + innerAngleB) + dHipOffset, 0, 180, USMIN, USMAX));
    pwm.writeMicroseconds(dKnee, map(((outerAngleKneeB + innerAngleKneeA)) + dKneeOffset, 0, 180, USMIN, USMAX));
    pwm.writeMicroseconds(dAnkle, map((kneeAngle) + dAnkleOffset, 0, 180, USMIN, USMAX));
  }
}

float pytherm(float sidea, float sideb)
{ // solves for side c
  float sidec = 0;
  sidec = (pow(sidea, 2) + pow(sideb, 2));
  sidec = sqrt(sidec);
  return sidec;
}

float pythermhypt(float sidea, float sidec)
{ // solves for side b
  float sideb = 0;
  sideb = ((pow(sidec, 2)) - pow(sidea, 2));

  sideb = sqrt(sideb);
  return sideb;
}

float raddec(float rad)
{
  rad = rad * (180 / PI);
  return rad;
}

float loc(float a, float b, float c)
{
  // this finds the angle for c
  float anglec = ((pow(a, 2) + pow(b, 2)) - pow(c, 2)) / (2 * a * b);
  anglec = acos(anglec);
  anglec = raddec(anglec);
  return anglec;
}

float decrad(float deg)

{
  deg = deg * (PI / 180);
  return deg;
}

void updateAll(){
  aLR.update();
  aHeight.update();
  aFB.update();

  bLR.update();
  bHeight.update();
  bFB.update();

  cLR.update();
  cHeight.update();
  cFB.update();

  dLR.update();
  dHeight.update();
  dFB.update();
}