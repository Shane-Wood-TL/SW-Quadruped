#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


float pytherm(float sidea, float sideb); //returns hypotenuse c
float raddec(float rad); //radians to degress
float loc(float a, float b, float c); // law of cosines, returns angle c in degress 
float pythermhypt(float sidea, float sidec); //returns side b


void mainKinematics(float xH , float xFB, float xLR, int hipMotor, int kneeMotor, int ankleMotor, bool inverted);
void updateAll();
void move(int forwardLegHip, int timePeriod, float ChangexH, float ChangexFB, float ChangexLR);


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define USMIN  544 //min value given from arduino lib
#define USMAX  2400 //max value given from arduino lib
#define SERVO_FREQ 50

//constant distances
float aLength = 85; // upper leg length more red
float bLength = 101.5; // lower leg length more purple
float Ldis = 110;

//motor definitions
int aHip = 0; //75 is 90
int aKnee = 1; // 0 is straight down
int aAnkle = 2;// 0 is correct

int bHip = 3; //80 is 90
int bKnee = 4;// 0 is 0
int bAnkle = 5;// 0 is 0

int cHip = 6; //105 is 90
int cKnee = 7; //150 is 0
int cAnkle = 8; //150 is 0

int dHip = 9; //105 is 90
int dKnee = 10;//150 is 0
int dAnkle = 11; //170 is 0


 //assigned inputs
float AxH = 120;
float AxLR = 0;
float AxFB = 0;

float BxH = 120;
float BxLR = 0;
float BxFB = 0;

float CxH = 120;
float CxLR = 0;
float CxFB = 0;

float DxH = 120;
float DxLR = 0;
float DxFB = 0;

//assigned angles
float givenX = 0;
float givenY = 0;
float givenZ = 0;

//inputs from sensors
//sonar
float dis[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//gyro 
float Gx = 0;
float Gy = 0;
float Gz = 0;

//acceromator
float Ax = 0;
float Ay = 0;
float Az = 0;

//Total angle pos; gyro + assigned
float TxA = 0; 
float TyA = 0;
float TzA = 0;

//World position variables; taking the angles of the bot and applying them to each leg value
float WAxH = 130;
float WAxLR = -10;
float WAxFB = 0;

float WBxH = 150;
float WBxLR = -10;
float WBxFB = 0;

float WCxH = 130;
float WCxLR = -10;
float WCxFB = 0;

float WDxH = 150;
float WDxLR = -10;
float WDxFB = 0;



int longtime = 75;
int quicktime = 30;
int upDis = 35;

int backLength = 60;


void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600);

   pwm.begin();
   pwm.setOscillatorFrequency(27000000);
   pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
   delay(10);


//starting poistions for walkings

//move(aHip, longtime, 0,  backLength, 0); //120, 13, 0
//move(bHip, longtime, 0,  backLength, 0);
//move(cHip, longtime, 0,  backLength, 0);
//move(dHip, longtime, 0,  backLength, 0);


   
}

void loop() {

/*
mainKinematics(130, 0, 0, aHip, aKnee, aAnkle, false);
mainKinematics(130, 0, 0, bHip, bKnee, bAnkle, false);
mainKinematics(130, 0, 0, cHip, cKnee, cAnkle, true);
mainKinematics(130, 0, 0, dHip, dKnee, dAnkle, true);
*/
move(cHip, longtime, upDis,  backLength, 0);
move(cHip, longtime, -upDis, -backLength, 0); //120, 13, 0






move(bHip, longtime, upDis,  backLength, 0);
move(bHip, longtime, -upDis, -backLength, 0); //120, 13, 0


delay(30);

move(aHip, longtime, upDis,  backLength, 0);
move(aHip, longtime, -upDis,  -backLength, 0); //120, 13, 0


move(dHip, longtime, upDis,  backLength, 0);
move(dHip, longtime, -upDis,  -backLength, 0);


delay(30);











}



float pytherm(float sidea, float sideb){ //solves for side c
  float sidec = 0;
  sidec = (pow(sidea, 2) + pow(sideb, 2));
  sidec = sqrt(sidec); 
  return sidec;
}

float pythermhypt(float sidea, float sidec){ //solves for side b
  float sideb = 0;
  sideb = ((pow(sidec, 2))-pow(sidea, 2));

  sideb = sqrt(sideb); 
  return sideb;
}

float raddec(float rad){
    rad = rad*(180/PI);
    return rad;
}

float loc(float a, float b, float c){
    // this finds the angle for c
    float anglec = (pow(c,2) - (pow(a,2) + pow(b,2))) / (-2*a*b);
    anglec = acos(anglec); 
    anglec = raddec(anglec); 
    return anglec;
}


// given height, forward/back, left/right, hip motor num, knee motor num, ankle motor num, bool if on the other side.
void mainKinematics(float xH , float xFB, float xLR, int hipMotor, int kneeMotor, int ankleMotor, bool inverted){


  
  /*
  this is intended to make the kinematic system into a single function that works for all legs
  it could be done with fewer inputs, however this is going to make it was easier for me 
  */

   //hip angle
  float innerLegLength = pytherm(xH, Ldis+xLR);
  //now must solve for 1st purple angle
  //using cos, a/h
  float innerAngleA = acos(xH/innerLegLength); //adj / hyp
  
  innerAngleA = raddec(innerAngleA);
  
  //now for the second triangle of the first part
  float modLegL = pythermhypt (Ldis, innerLegLength); //most outer triangle
  //now must solve for the 2nd purple angle
  float innerAngleB = acos(Ldis/innerLegLength);// adjacent and hyp cah soh
  innerAngleB = raddec(innerAngleB);

//Serial.println(innerAngleA);
//Serial.println(innerAngleB);
  //everything up to this point works still 

  //write the 2 values the the hip motor at the end now

 // time for stage 2
 float modLegLL = pytherm (xFB, modLegL);
 // solve for this angle aswell
 
  float innerAngleKneeA = acos(modLegL/modLegLL); //cah
  innerAngleKneeA = raddec(innerAngleKneeA);

 //solve for the other triangle
 // LAW OF COSINES NEEDED, all sides found


  //float modLegLL= xH;
  float outerAngleKneeB = loc(aLength, modLegLL, bLength);
  //Serial.println(outerAngleKneeB);
  //now finally the last value can be passed to the ankle servo
  //using loc to find the angle for the ankle

  float kneeAngle = loc(aLength,bLength, modLegLL);

  //Serial.println(kneeAngle);
 //everything looks fine within the code, however i think something is still broken
  if (inverted == false){
    if(hipMotor == 0){
      //offsets for a

      pwm.writeMicroseconds(hipMotor, map(165-(innerAngleA+innerAngleB), 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(kneeMotor, map((outerAngleKneeB+innerAngleKneeA), 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(ankleMotor, map(kneeAngle+10, 0, 180, USMIN, USMAX));

      //Serial.println(innerAngleKneeA+outerAngleKneeB);

    }else{
      //offsets for b
      pwm.writeMicroseconds(hipMotor, map(175-(innerAngleA+innerAngleB), 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(kneeMotor, map((outerAngleKneeB+innerAngleKneeA), 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(ankleMotor, map(kneeAngle, 0, 180, USMIN, USMAX));

    }
  }
  else if (inverted == true){
    if(hipMotor == 6){
      //offsets for c
      pwm.writeMicroseconds(hipMotor, map((innerAngleA+innerAngleB)+15, 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(kneeMotor, map(155-(outerAngleKneeB+innerAngleKneeA), 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(ankleMotor, map(150-kneeAngle, 0, 180, USMIN, USMAX));

    }else{
      //offsets for d
      pwm.writeMicroseconds(hipMotor, map((innerAngleA+innerAngleB)+12, 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(kneeMotor, map(150-(outerAngleKneeB+innerAngleKneeA), 0, 180, USMIN, USMAX));
      pwm.writeMicroseconds(ankleMotor, map(155-kneeAngle, 0, 180, USMIN, USMAX));

    }

  }else{
    //Serial.print("something is very wrong");
  }
  return;

}

void updateAll(){
  mainKinematics(WAxH, WAxFB, WAxLR, aHip, aKnee, aAnkle, false);
  //Serial.println(WAxH);
  mainKinematics(WBxH, WBxFB, WBxLR, bHip, bKnee, bAnkle, false);
  mainKinematics(WCxH, WCxFB, WCxLR, cHip, cKnee, cAnkle, true);
  mainKinematics(WDxH, WDxFB, WDxLR, dHip, dKnee, dAnkle, true);
  return;


}

void move(int forwardLegHip, int timePeriod, float ChangexH, float ChangexFB, float ChangexLR){
  int cout = timePeriod/5;
  switch (forwardLegHip)
  {
  case 0:

  for (int i = 0; i < cout; i++)
  {
    //main leg moving
    WAxH = WAxH +(ChangexH/cout);
    WAxFB = WAxFB + (ChangexFB/cout);
    WAxLR = WAxLR + (ChangexLR/cout);
    updateAll();
    //Serial.println(i);
  }
  
    
    break;
  
  case 3:
    for (int i = 0; i < cout; i++)
    {
        //main leg moving
        WBxH = WBxH +(ChangexH/cout);
        WBxFB = WBxFB + (ChangexFB/cout);
        WBxLR = WBxLR + (ChangexLR/cout);
        updateAll();
        //Serial.println(i);
    }

  break;

  case 6:
    for (int i = 0; i < cout; i++)
    {
      //main leg moving
      WCxH = WCxH +(ChangexH/cout);
      WCxFB = WCxFB + (ChangexFB/cout);
      WCxLR = WCxLR + (ChangexLR/cout);
      updateAll();
      //Serial.println(i);
    }

  break;

  case 9:
    for (int i = 0; i < cout; i++)
    {
      //main leg moving
      WDxH = WDxH +(ChangexH/cout);
      WDxFB = WDxFB + (ChangexFB/cout);
      WDxLR = WDxLR + (ChangexLR/cout);
      updateAll();
      //Serial.println(i);
    }

  break; 

  default:
    
    break;
  }
  
}