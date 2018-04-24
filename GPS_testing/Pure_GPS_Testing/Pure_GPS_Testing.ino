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

// our struct to send stuff
struct GPSinfo{
  double latitude;    
  double longitude; 
  double speed; 
  unsigned long course;
  unsigned long altitude;
};

// Struct for displaying info
struct GPSinfo displayPackage;

// timer variable
unsigned long previousMillis = 0;   // holds previous time we displayed

void setup() {
  Serial.begin(9600);
  Serial.println(F("Welcome to the GPS transceiving unit. This is the laptop module"));

  // set up our GPS
  gpss.begin(GPSBaud);
}

void loop() 
{
  // gps own stuff
  while (gpss.available() > 0)
  {
    if (gps.encode(gpss.read()))
    {
      displayPackage.latitude = gps.location.lat();
      displayPackage.longitude = gps.location.lng();
      displayPackage.speed = gps.speed.mph();
      displayPackage.course = gps.course.value();
      displayPackage.altitude = gps.altitude.value();
    }
  }
  

  // holds current time
    unsigned long currentMillis = millis();
  // this will display if we haven't displayed for a second
    if (currentMillis - previousMillis >= 1000)
    {
      // displaying our own coordinates
      Serial.print(F("Computer Unit:   "));
      
      Serial.print(F("Latitude: "));
      Serial.print(displayPackage.latitude, 6);
      Serial.print(F("  "));

      Serial.print(F("Longitude: "));
      Serial.print(displayPackage.longitude, 6);
      Serial.print(F("  "));

      Serial.print(F("Speed: "));
      Serial.print(displayPackage.speed);
      Serial.print(F(" mph  "));

      double realCourse = displayPackage.course/100.0;
      Serial.print(F("Course: "));
      Serial.print(realCourse);
      Serial.print(F(" degrees  "));

      double realAltitude = displayPackage.altitude/100.0;
      Serial.print(F("Altitude: "));
      Serial.print(realAltitude);
      Serial.println(F(" meters  "));

      // update our timer variable
      previousMillis = currentMillis;
    }
}

