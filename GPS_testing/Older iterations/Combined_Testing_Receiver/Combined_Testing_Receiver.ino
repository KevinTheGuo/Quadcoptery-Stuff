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

// our struct to send stuff
struct GPSinfo{
  double latitude;    
  double longitude; 
  double speed; 
  unsigned long course;
  unsigned long altitude;
};

struct GPSinfo radioPackage;

// timer variable
unsigned long previousMillis = 0;   // holds previous time we displayed

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
  radio.setPALevel(RF24_PA_MAX); // we might need to do this or not

  // open radio pipes
//  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);
  radio.startListening();

  // set up our GPS
//  gpss.begin(GPSBaud);
}

void loop() 
{
  if(radio.available())
  {
  //  Serial.println(F("We got something!"));
    while(radio.available())
    {
      radio.read(&radioPackage, sizeof(radioPackage));
    }
  }

  // holds current time
    unsigned long currentMillis = millis();
  // this will display if we haven't displayed for a second
    if (currentMillis - previousMillis >= 1000)
    {
      // display!
      Serial.print(F("Latitude: "));
      Serial.print(radioPackage.latitude, 6);
      Serial.print(F("  "));

      Serial.print(F("Longitude: "));
      Serial.print(radioPackage.longitude, 6);
      Serial.print(F("  "));

      Serial.print(F("Speed: "));
      Serial.print(radioPackage.speed);
      Serial.print(F(" mph  "));

      double realCourse = radioPackage.course/100.0;
      Serial.print(F("Course: "));
      Serial.print(realCourse);
      Serial.print(F(" degrees  "));

      double realAltitude = radioPackage.altitude/100.0;
      Serial.print(F("Altitude: "));
      Serial.print(realAltitude);
      Serial.println(F(" meters  "));

      previousMillis = currentMillis;
    }
}

