#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

const int j1_X = A0;
const int j1_Y = A1;
const int j2_X = A2;
const int j2_Y = A3;

const int CE = 9;
const int CSN = 10;
const int SCK = 13;
const int MOSI = 11;
const int MISO = 12;

const int sw1 = 2;
const int sw2 = 3;
const int eStop = 4;
const int sw3 = 5;
const int sw4 = 6;

const int j1_B = 7;
const int j2_B = 8;


float j1_X_angle, j1_Y_angle, j2_X_angle, j2_Y_angle;
int sw1V, sw2V, eStopV, sw3V, sw4V, j1_BV, j2_BV;

void setup() {
  pinMode(sw1, INPUT_PULLUP);
  pinMode(sw2, INPUT_PULLUP);
  pinMode(eStop, INPUT_PULLUP);
  pinMode(sw3, INPUT_PULLUP);
  pinMode(sw4, INPUT_PULLUP);

  pinMode(j1_B, INPUT_PULLUP);
  pinMode(j2_B, INPUT_PULLUP);

}

void loop() {
  j1_X_angle = digitalRead(j1_X);
  j1_Y_angle = analogRead(j1_Y);
  j2_X_angle = analogRead(j2_X);
  j2_Y_angle = analogRead(j2_Y);

  sw1V = digitalRead(sw1);
  sw2V = digitalRead(sw2);
  eStopV = digitalRead(eStop);
  sw3V = digitalRead(sw3);
  sw4V = digitalRead(sw4);

  

}

