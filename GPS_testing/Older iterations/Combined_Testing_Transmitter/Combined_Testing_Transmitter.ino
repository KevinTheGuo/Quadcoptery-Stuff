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


// our struct to send stuff
struct GPSinfo{
  double latitude;    
  double longitude; 
  double speed; 
  unsigned long course;
  unsigned long altitude;
};

struct GPSinfo radioPackage;

// husky is for GPS writing, sammy is for laptop writing
byte addresses[][6] = {"1Node","2Node"};

void setup() {
  Serial.begin(9600);
  Serial.println(F("Welcome to the GPS transceiving unit. This is the GPS module, we are transmitting"));

  // set our radio stuff
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_MAX); // we might need to do this or not

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

    radioPackage.latitude = gps.location.lat();
    radioPackage.longitude = gps.location.lng();
    radioPackage.speed = gps.speed.mph();
    radioPackage.course = gps.course.value();
    radioPackage.altitude = gps.altitude.value();
    
   if (!radio.write(&radioPackage, sizeof(radioPackage)))
   {
     Serial.println(F("oopsies"));
   }
}

