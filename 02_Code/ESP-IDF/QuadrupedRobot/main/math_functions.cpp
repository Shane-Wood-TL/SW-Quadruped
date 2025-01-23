#include "../include/all_includes.h"
#define PI 3.14

//------------------------------------------------------------------------------------------------
//finds hypotenuse of triangle
float pytherm(float sidea, float sideb)
{ // solves for side c
  float sidec = 0;
  sidec = (pow(sidea, 2) + pow(sideb, 2));
  sidec = sqrt(sidec);
  return sidec;
}


//------------------------------------------------------------------------------------------------
//finds leg of triangle
float pythermhypt(float sidea, float sidec)
{ // solves for side b
  float sideb = 0;
  sideb = ((pow(sidec, 2)) - pow(sidea, 2));

  sideb = sqrt(sideb);
  return sideb;
}


//------------------------------------------------------------------------------------------------
//converts radians to degrees
float raddec(float rad)
{
  rad = rad * (180 / PI);
  return rad;
}


//------------------------------------------------------------------------------------------------
//law of consines, returns angle across from c
float loc(float a, float b, float c)
{
  // this finds the angle for c
  float anglec = ((pow(a, 2) + pow(b, 2)) - pow(c, 2)) / (2 * a * b);
  anglec = acos(anglec);
  anglec = raddec(anglec);
  return anglec;
}


//------------------------------------------------------------------------------------------------
//converts degrees to radians
float decrad(float deg)

{
  deg = deg * (PI / 180);
  return deg;
}