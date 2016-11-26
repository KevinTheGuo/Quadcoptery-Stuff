#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include "RF24.h"

/* This sketch involves two arduino units communicating to each other through the RF24 library. One unit sends its location to the other. 
 *  This particular sketch is for the transmitter unit
 */

// const ints for the GPS pin
static const int RXPin = 4, TXPin = 3;

// the Radio object
RF24 radio(7,8);

// husky is for GPS writing, sammy is for laptop writing
byte addresses[][6] = {"husky","sammy"};

struct dataPackage{
  int oneNumber;
  char string[6];
  float pie;
};

struct dataPackage radioPackage;

void setup() {
  Serial.begin(9600);
  Serial.println(F("Welcome to the the husky transceiving unit!"));

  // set our radio stuff
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_LOW); // we might need to do this or not

  // open radio pipes
  radio.openWritingPipe(addresses[1]);
//  radio.openReadingPipe(1,addresses[0]);
  radio.stopListening();

  // do our struct stuff
  radioPackage.oneNumber = 42;
  strcpy(radioPackage.string, "Husky");
  radioPackage.pie = 3.141529;

}

void loop() {
    Serial.println(F("Sending our info"));
    
   if (!radio.write(&radioPackage, sizeof(radioPackage)))
   {
     Serial.println(F("oopsies"));
   }
}

