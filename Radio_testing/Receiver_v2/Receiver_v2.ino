#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include "RF24.h"

/* This sketch involves two arduino units communicating to each other through the RF24 library. We are testing the sending 
 *  and receiving of large packages
 */

// const ints for the GPS pin
static const int RXPin = 4, TXPin = 3;

// the Radio object
RF24 radio(7,8);

// husky is for GPS writing, sammy is for laptop writing
byte addresses[][6] = {"husky","sammy"};

// our struct
struct dataPackage{
  int oneNumber;
  char string[6];
  float pie;
};
struct dataPackage radioPackage;

void setup() {
  Serial.begin(9600);
  Serial.println(F("Welcome to the samoyed transceiving unit"));

  // set our radio stuff
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_LOW); // we might need to do this or not

  // open radio pipes
//  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);
  radio.startListening();
}

void loop() 
{
  char str[32];
  if(radio.available())
  {
    Serial.println(F("We got something!"));
    while(radio.available())
    {
      radio.read(&radioPackage, sizeof(radioPackage));
    }

    Serial.print(radioPackage.oneNumber);
    Serial.print(radioPackage.string);
    Serial.print(radioPackage.pie);
    Serial.println(" done");
  }
}


