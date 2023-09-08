#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


void showData();
void getData();

RF24 radio(16,9);
const byte thisSlaveAddress[5] = {'R','x','A','A','A'};

char dataReceived[10]; // this must match dataToSend in the TX
bool newData = false;


void setup() {

    Serial.begin(9600);
    SPI.begin(18, 8, 10, 16);
    Serial.println("SimpleRx Starting");
    radio.begin();
    radio.printDetails();
    delay(1000);
    radio.setDataRate( RF24_250KBPS );
    radio.openReadingPipe(1, thisSlaveAddress);
    radio.startListening();
}

//=============

void loop() {
    getData();
    showData();
    
}

//==============

void getData() {
    if ( radio.available() ) {
        radio.read( &dataReceived, sizeof(dataReceived) );
        newData = true;
    }
}

void showData() {
    if (newData == true) {
        Serial.print("Data received ");
        Serial.println(dataReceived);
        newData = false;
    }
}