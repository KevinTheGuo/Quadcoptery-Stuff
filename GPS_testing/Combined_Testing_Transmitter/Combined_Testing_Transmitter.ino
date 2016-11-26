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

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial gpss(RXPin, TXPin);

// the Radio object
RF24 radio(7,8);

// husky is for GPS writing, sammy is for laptop writing
byte addresses[][6] = {"1Node","2Node"};

void setup() {
  Serial.begin(9600);
  Serial.println(F("Welcome to the GPS transceiving unit. This is the GPS module, we are transmitting"));

  // set our radio stuff
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_LOW); // we might need to do this or not

  // open radio pipes
  radio.openWritingPipe(addresses[1]);
//  radio.openReadingPipe(1,addresses[0]);
  radio.stopListening();

  // set up our GPS
  gpss.begin(GPSBaud);
}

void loop() {
    while (gpss.available() > 0)
    if (gps.encode(gpss.read()))
    {
      transmitInfo();
    }

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("Uh ohz. Something is wrong. "));
    while(true);  // just stop it. end ourselves. nothing is worth it anymore.
  }
}

void transmitInfo()
{
  Serial.println(F("Sending our GPS info"));

//  unsigned long package = 0x0;
//  if (gps.location.isValid())
  
    uint16_t lat = gps.location.rawLat().deg;
    uint16_t lng = gps.location.rawLng().deg;
    unsigned long package = (lat << 16) | lng;                             // put our stuff together into one package 
  
    Serial.println(package);
    Serial.print(lat);
    Serial.print(",");
    Serial.println(lng);
    
   if (!radio.write(&package, sizeof(unsigned long)))
   {
     Serial.println(F("oopsies"));
   }
}

