#include <Arduino.h>
#include <Ramp.h>

void mainKinematics(float xH , float xFB, float xLR, int hipMotor, float xRot, float yRot, float zRot);

float pytherm(float sidea, float sideb); //returns hypotenuse c
float raddec(float rad); //radians to degress
float loc(float a, float b, float c); // law of cosines, returns angle c in degress 
float pythermhypt(float sidea, float sidec); //returns side b
float decrad(float deg); //degrees to radians
void all_90s();

void walk(int &Cycle, ramp &FB, ramp &Height, float time, float backDistance, float upDistance, bool d);
void WalkF(float yRot, float zRot);
void WalkLR(float yRot, float zRot, bool d);
void resetAll();

void turning(int &Cycle, ramp &LR, ramp &Height, float timee, float LRDistance, float upDistance, bool direction);
void turner(float yRot, float zRot);

void updateAll();


