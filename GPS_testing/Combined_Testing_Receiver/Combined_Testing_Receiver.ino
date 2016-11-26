#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include "RF24.h"

/* This sketch involves two arduino units communicating to each other through the RF24 library. One unit sends its location to the other. 
 *  This particular sketch is for the transmitter unit
 */

// const ints for the GPS pin
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
/*
// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial gpss(RXPin, TXPin);
*/

// the Radio object
RF24 radio(7,8);

// husky is for GPS writing, sammy is for laptop writing
byte addresses[][6] = {"1Node","2Node"};

void setup() {
  Serial.begin(9600);
  Serial.println(F("Welcome to the GPS transceiving unit. This is the laptop module, we are receiving"));

  // set our radio stuff
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_LOW); // we might need to do this or not

  // open radio pipes
//  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);
  radio.startListening();

  // set up our GPS
//  gpss.begin(GPSBaud);
}

void loop() 
{
  unsigned long package;
  if(radio.available())
  {
    Serial.println(F("We got something!"));
    while(radio.available())
    {
      radio.read(&package, sizeof(unsigned long));
    }
    displayInfo(package);
  }
}

void displayInfo(unsigned long package)
{
  uint16_t lat = (package >> 16) & 0xFFFF;
  uint16_t lng = package & 0xFFFF;

  Serial.print(F("Location: ")); 
  Serial.print(lat);
  Serial.print(F(",")); 
  Serial.println(lng);

  Serial.println(package);
}

