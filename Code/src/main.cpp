#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <utility/imumaths.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>





float pytherm(float sidea, float sideb); //returns hypotenuse c
float raddec(float rad); //radians to degress
float loc(float a, float b, float c); // law of cosines, returns angle c in degress 
float pythermhypt(float sidea, float sidec); //returns side b
float decrad(float deg); //degrees to radians




void mainKinematics(float xH , float xFB, float xLR, int hipMotor, float xRot, float yRot, float zRot);




Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

#define USMIN  544 //min value given from arduino lib
#define USMAX  2400 //max value given from arduino lib
#define SERVO_FREQ 50

//constant distances
const float aLength = 95; // upper leg length more red
const float bLength = 95; // lower leg length more purple
const float Ldis = 45;

//motor definitions
const int aHip = 0; //
const int aKnee = 1; // 
const int aAnkle = 2;// 

const int bHip = 3; //
const int bKnee = 4;//
const int bAnkle = 5;// 

const int cHip = 6; //
const int cKnee = 7; //
const int cAnkle = 8; //

const int dHip = 9; //
const int dKnee = 10;//
const int dAnkle = 11; //


const int yHalfDis = 85;
const int zHalfDis = 125;

float xAngle;
float yAngle;
float zAngle;



float testHeight = 150;
float testLR = 0;
float testFB = 0;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
   Wire.begin(10,8); //SDA, SCL
   //Wire.begin();
   pwm.begin();
   pwm.setOscillatorFrequency(27000000);
   pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
   delay(10);  

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);
 delay(1000);  
    mainKinematics(testHeight, testFB, testLR, aHip,0,0,0);
    mainKinematics(testHeight, testFB, testLR, bHip,0,0,0);
    mainKinematics(testHeight, testFB, testLR, cHip,0,0,0);
    mainKinematics(testHeight, testFB, testLR, dHip,0,0,0);

 

}

void loop() {
  int moveToHeight = 150;
  // mainKinematics(125, -20, testLR,aHip,0,0,0);
  // mainKinematics(125, -20, testLR,cHip,0,0,0);

  // mainKinematics(100, 0, testLR,bHip,0,0,0);
  // mainKinematics(100, 0, testLR,dHip,0,0,0);
   delay(200);

  // mainKinematics(100, 0, testLR, aHip,0,0,0);
  // mainKinematics(100, 0, testLR,cHip,0,0,0);
  // mainKinematics(125, -20, testLR,bHip,0,0,0);
  //   mainKinematics(125, -20, testLR,dHip,0,0,0);
  // delay(200);

  sensors_event_t event;
  bno.getEvent(&event);

  Serial.print("X: ");
  Serial.print(event.orientation.x, 0);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 0);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 0);
  Serial.println("");
int yRot= event.orientation.y, zRot = event.orientation.z;
// event.orientation.y
mainKinematics(testHeight, testFB, testLR, aHip,0,yRot,zRot);
mainKinematics(testHeight, testFB, testLR, bHip,0,yRot,zRot);
mainKinematics(testHeight, testFB, testLR, cHip,0,yRot,zRot);
mainKinematics(testHeight, testFB, testLR, dHip,0,yRot,zRot);



  // mainKinematics(testHeight, testFB, testLR, aHip,0,-(event.orientation.y),event.orientation.z);
  //   mainKinematics(testHeight, testFB, testLR, bHip,0,event.orientation.y,event.orientation.z);
  //   mainKinematics(testHeight, testFB, testLR, cHip,0,event.orientation.y,-(event.orientation.z));
  //   mainKinematics(testHeight, testFB, testLR, dHip,0,-(event.orientation.y),-(event.orientation.z));
    //       mainKinematics(testHeight, testFB, testLR, aHip,0,0,0);
    // mainKinematics(testHeight, testFB, testLR, bHip,0,0,0);
    // mainKinematics(testHeight, testFB, testLR, cHip,0,0,0);
    // mainKinematics(testHeight, testFB, testLR, dHip,0,0,0);

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
  float anglec = ((pow(a,2) + pow(b,2)) - pow(c,2)) / (2*a*b);
  anglec = acos(anglec); 
  anglec = raddec(anglec); 
  return anglec;
}

float decrad(float deg){
  deg = deg*(PI/180);
  return deg;
}


// given height, forward/back, left/right, hip motor num, knee motor num, ankle motor num, bool if on the other side.
void mainKinematics(float xH , float xFB, float xLR, int hipMotor, float xRot, float yRot, float zRot){
float innerAngleKneeA, kneeAngle, outerAngleKneeB;//cah
float zMultiplier = 1.25;
float yMultiplier = 1.25;
 //Serial.print("this is running");

  //angle code (no idea how s rotation works so for now it is going to be ignored)
  float yRotR = decrad(abs(yRot));
  //float yRotR = decrad(abs(yRot));

  float yAddition = yMultiplier *yHalfDis*tan(yRotR);

  if((yAddition < xH) && (hipMotor == 6 || hipMotor == 3)){
    if(yRot< 0){
      xH -= yAddition;
    }else{
      xH += yAddition;
    }
   
  }else if((yAddition < xH) && (hipMotor == 0|| hipMotor == 9)){
      if(yRot< 0){
        xH += yAddition;
      }else{
        xH -= yAddition;
      }
  }

  float zRotR = decrad(abs(zRot));
  float zAddition = zHalfDis*tan(zRotR);
  if((zAddition < xH) && (hipMotor == 0 || hipMotor == 3)){
    if(zRot< 0){
      xH += zAddition*zMultiplier;
    }else{
      xH -= zAddition*zMultiplier;
    }
   
  }else if((zAddition < xH) && (hipMotor == 6|| hipMotor == 9)){
      if(zRot< 0){
        xH -= zAddition*zMultiplier;
      }else{
        xH += zAddition*zMultiplier;
      }
  }

  
  /*
  this is intended to make the kinematic system into a single function that works for all legs
  it could be done with fewer inputs, however this is going to make it was easier for me 
  */

   //hip angle
  


  float innerLegLength = pytherm(xH, Ldis+xLR);
  //now must solve for 1st purple angle
  //using cos, a/h
  //now for the second triangle of the first part
  float modLegL = pythermhypt (Ldis, innerLegLength); //most outer triangle

  float innerAngleA = acos(xH/innerLegLength); //adj / hyp
  
  innerAngleA = raddec(innerAngleA);
  

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
  if(xFB <= 0){
    xFB = abs(xFB);

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
  

    if(hipMotor == 0){
          //offsets for a
          
          pwm.writeMicroseconds(aHip, map((innerAngleA+innerAngleB)-20, 0, 180, USMIN, USMAX));
          pwm.writeMicroseconds(aKnee, map((outerAngleKneeB+innerAngleKneeA)+10, 0, 180, USMIN, USMAX));
          pwm.writeMicroseconds(aAnkle, map((kneeAngle)-10, 0, 180, USMIN, USMAX));

          //Serial.println(innerAngleKneeA+outerAngleKneeB);

    }else if(hipMotor == 3){
          //offsets for b
          pwm.writeMicroseconds(bHip, map((180-(innerAngleA+innerAngleB))-5, 0, 180, USMIN, USMAX));
          pwm.writeMicroseconds(bKnee, map((180-(outerAngleKneeB+innerAngleKneeA)), 0, 180, USMIN, USMAX));
          pwm.writeMicroseconds(bAnkle, map((191-kneeAngle), 0, 180, USMIN, USMAX));

    }else if(hipMotor == 6){
          //offsets for c
          pwm.writeMicroseconds(cHip, map((180-(innerAngleA+innerAngleB))-5, 0, 180, USMIN, USMAX));
          pwm.writeMicroseconds(cKnee, map((180-(outerAngleKneeB+innerAngleKneeA)), 0, 180, USMIN, USMAX));
          pwm.writeMicroseconds(cAnkle, map((180-kneeAngle), 0, 180, USMIN, USMAX));

    }else if(hipMotor == 9){
          //offsets for d
          pwm.writeMicroseconds(dHip, map((innerAngleA+innerAngleB)-5, 0, 180, USMIN, USMAX));
          pwm.writeMicroseconds(dKnee, map(((outerAngleKneeB+innerAngleKneeA))+13, 0, 180, USMIN, USMAX));
          pwm.writeMicroseconds(dAnkle, map((kneeAngle)-12, 0, 180, USMIN, USMAX));
    }
    }else{
      float innerAngleKneeA = acos(modLegL/modLegLL); //cah
      float innerAngleKneeB = loc(aLength, modLegLL, bLength);
      float newKneeAngle = 90+(innerAngleB-innerAngleA);

      //need to write 90 + (b-a)
      float kneeAngle = loc(aLength,bLength, modLegLL);
        if(hipMotor == 0){
          //offsets for a
          pwm.writeMicroseconds(aHip, map((innerAngleA+innerAngleB)-10, 0, 180, USMIN, USMAX));
          pwm.writeMicroseconds(aKnee, map(180-newKneeAngle, 0, 180, USMIN, USMAX));
          pwm.writeMicroseconds(aAnkle, map((kneeAngle)-10, 0, 180, USMIN, USMAX));

          //Serial.println(innerAngleKneeA+outerAngleKneeB);

    }else if(hipMotor == 3){
          //offsets for b
          pwm.writeMicroseconds(bHip, map((180-(innerAngleA+innerAngleB))-5, 0, 180, USMIN, USMAX));
          pwm.writeMicroseconds(bKnee, map((newKneeAngle), 0, 180, USMIN, USMAX));
          pwm.writeMicroseconds(bAnkle, map((191-kneeAngle), 0, 180, USMIN, USMAX));

    }else if(hipMotor == 6){
          //offsets for c
          pwm.writeMicroseconds(cHip, map((180-(innerAngleA+innerAngleB))-5, 0, 180, USMIN, USMAX));
          pwm.writeMicroseconds(cKnee, map((newKneeAngle), 0, 180, USMIN, USMAX));
          pwm.writeMicroseconds(cAnkle, map((180-kneeAngle), 0, 180, USMIN, USMAX));

    }else if(hipMotor == 9){
          //offsets for d
          pwm.writeMicroseconds(dHip, map((innerAngleA+innerAngleB)+2, 0, 180, USMIN, USMAX));
          pwm.writeMicroseconds(dKnee, map((180-newKneeAngle)+10, 0, 180, USMIN, USMAX));
          pwm.writeMicroseconds(dAnkle, map((kneeAngle)-12, 0, 180, USMIN, USMAX));
    } 
    }
}

  // int moveToHeight = 125;
  // for(int i = 100; i<moveToHeight; i++){
  //   mainKinematics(i,-(i-100), testLR, aHip,0,0,0);
  //   mainKinematics(10+i, -(i-100), testLR, cHip,0,0,0);

  // }
  // mainKinematics(100, 0, testLR, aHip,0,0,0);
  // mainKinematics(100, 0, testLR, cHip,0,0,0);
  // delay(2);
  // for(int i = 100; i<moveToHeight; i++){
  //   mainKinematics(i, -(i-100), testLR, bHip,0,0,0);
  //   mainKinematics((i), -(i-100), testLR, dHip,0,0,0);
  // }
  // mainKinematics(100, 0, testLR, bHip,0,0,0);
  // mainKinematics(100, 0, testLR, dHip,0,0,0);
  // delay(2);