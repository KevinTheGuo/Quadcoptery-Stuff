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

// two structs. one for displaying, one for receiving
struct GPSinfo radioPackage;
struct GPSinfo displayPackage;

// timer variable
unsigned long previousMillis = 0;   // holds previous time we displayed

// the Radio object
RF24 radio(7,8);

// husky is for GPS writing, sammy is for laptop writing
byte addresses[][6] = {"1Node","2Node"};

void setup() {
  Serial.begin(9600);
  Serial.println(F("Welcome to the GPS transceiving unit. This is the laptop module"));

  // set our radio stuff
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_MAX); // we might need to do this or not

  radio.enableAckPayload();                     // Allow optional ack payloads
  radio.enableDynamicPayloads();                // Ack payloads are dynamic payloads

  // open radio pipes
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);
  radio.startListening();
  radio.writeAckPayload(1,&displayPackage,sizeof(displayPackage));  

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
  
  recieveInfo();

  // holds current time
    unsigned long currentMillis = millis();
  // this will display if we haven't displayed for a second
    if (currentMillis - previousMillis >= 1000)
    {
      // update our ack package
      radio.writeAckPayload(1,&displayPackage,sizeof(displayPackage));
      
      // displaying the mobile unit coordinates
      Serial.print(F("Mobile Unit:   "));
      
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

      realCourse = displayPackage.course/100.0;
      Serial.print(F("Course: "));
      Serial.print(realCourse);
      Serial.print(F(" degrees  "));

      realAltitude = displayPackage.altitude/100.0;
      Serial.print(F("Altitude: "));
      Serial.print(realAltitude);
      Serial.println(F(" meters  "));

      // display the distance between ourselves
      double realDist = distCalc(displayPackage.latitude, displayPackage.longitude, radioPackage.latitude, radioPackage.longitude); 
      Serial.print(F("Distance between units: "));
      Serial.print(realDist);
      Serial.println(F(" meters  "));

      // update our timer variable
      previousMillis = currentMillis;
    }
}


void recieveInfo()
{  
  if(radio.available())
  {
  //  Serial.println(F("We got something!"));
    while(radio.available())
    {
      radio.read(&radioPackage, sizeof(radioPackage));
    }
  }
}

double distCalc(double lat1, double lng1, double lat2, double lng2)
{
    double distLat;
    double distLon;
    distLat = (lat2 - lat1);
    distLon = (lng2 - lng1);
    distLat = distLat / 57.29577951;
    distLon = distLon / 57.29577951; 
    double vA;
    double vC;
    double distance;

    vA = sin(distLat/2) * sin(distLat/2) + cos(lat1) * cos(lat2) * sin(distLon/2) * sin(distLon/2);
    vC = 2 * atan2(sqrt(vA),sqrt(1-vA));
    distance = 6371 * vC;
    return distance*1000.0;
}
