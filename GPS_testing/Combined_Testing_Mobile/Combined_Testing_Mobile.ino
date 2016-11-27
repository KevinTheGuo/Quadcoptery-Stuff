#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <LCD.h>                  // LCD library
#include <LiquidCrystal_I2C.h>    // another LCD library
#include <Wire.h>  
#include "RF24.h"

// random LCD stuff
LiquidCrystal_I2C  lcd(0x3F,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified module

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

// timer variable
unsigned long previousMillis = 0;   // holds previous time we displayed
int transmitFailed = 0;

// our struct to send stuff
struct GPSinfo{
  double latitude;    
  double longitude; 
  double speed; 
  unsigned long course;
  unsigned long altitude;
};

struct GPSinfo radioPackage;
struct GPSinfo recievePackage;

// husky is for GPS writing, sammy is for laptop writing
byte addresses[][6] = {"1Node","2Node"};

void setup() {
  Serial.begin(9600);
  Serial.println(F("Welcome to the GPS transceiving unit. This is the mobile module"));

  // set our radio stuff
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_MAX); // we might need to do this or not

  radio.enableAckPayload();                     // Allow optional ack payloads
  radio.enableDynamicPayloads();                // Ack payloads are dynamic payloads

  // open radio pipes
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1,addresses[0]);
  radio.stopListening();

  // set up our GPS
  gpss.begin(GPSBaud);

  LCDstart(); // lcd start routine
}

void loop() {
  while (gpss.available() > 0)
  {
    if (gps.encode(gpss.read()))
    {
      Serial.println(F("We encoded our gps stuff"));
    }
  }

  // holds current time
    unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 1000)
  {
    LCDupdate();
    transmitInfo();
    previousMillis = currentMillis;
  }
  
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("Uh ohz. Something is terribly wrong. "));
    while(true);  // just stop it. end ourselves. nothing is worth it anymore.
  }
}

void transmitInfo()
{
  radio.stopListening();
  Serial.println(F("Sending our GPS info"));

//  unsigned long package = 0x0;
//  if (gps.location.isValid())

    radioPackage.latitude = gps.location.lat();
    radioPackage.longitude = gps.location.lng();
    radioPackage.speed = gps.speed.mph();
    radioPackage.course = gps.course.value();
    radioPackage.altitude = gps.altitude.value();
    
   if (radio.write(&radioPackage, sizeof(radioPackage)))
   {
      transmitFailed = 0;
      if(radio.available())    
      {                             
        while(radio.available())    // If an ack with payload was received
        {                      
              radio.read(&recievePackage, sizeof(recievePackage));                  // Read it, and display the response time
        }
        Serial.print(F("Got a response!!!!")); 
      }
      else    // If nothing in the buffer, we got an ack but it is blank
      {
        Serial.print(F("Got blank response")); 
      }
   }
   else
   {
     Serial.println(F("transmitInfo didn't work"));
     transmitFailed = 1;
   }
}

// LCDstart function
void LCDstart()
{
  Serial.println("doing lcd stuff"); 
  lcd.begin(16, 2);     // initialize LCD
  lcd.setBacklightPin(3,POSITIVE);      // initialize backlight
  lcd.backlight(); // turn backlight on
  
  lcd.clear();      // clear
  lcd.setCursor(1,0);       // display title and stuff
  lcd.print("Welcome to the");
  lcd.setCursor(0,1);
  lcd.print("GPS Locator 3001");
  delay(2000);

  lcd.clear();      // clear for next display

  lcd.home();
  lcd.print("LNG");
  lcd.setCursor(8,0);
  lcd.print("LAT");
  lcd.setCursor(0,1);
  lcd.print("SPD");
  lcd.setCursor(5,1);
  lcd.print("HD");
  lcd.setCursor(10,1);
  lcd.print("DST");

  // the first round of updates
  LCDupdate();
  Serial.println("done with LCD stuff");
  return;
}

void LCDupdate()
{  
  lcd.setCursor(3,0);
  lcd.print("     ");
  lcd.setCursor(11,0);
  lcd.print("     ");
  lcd.setCursor(3,1);
  lcd.print("  ");
  lcd.setCursor(7,1);
  lcd.print("   ");
  lcd.setCursor(13,1);
  lcd.print("   ");
  
  lcd.setCursor(3,0);
  lcd.print(radioPackage.longitude);
  lcd.setCursor(11,0);
  lcd.print(radioPackage.latitude);
  lcd.setCursor(3,1);
  int realSpeed = radioPackage.speed;
  lcd.print(realSpeed);
  lcd.setCursor(7,1);
  int realCourse = radioPackage.course/100;
  lcd.print(realCourse);

  // find our distance
  lcd.setCursor(13,1);
  if (transmitFailed)
  {
    lcd.print("N/A");
  }
  else
  {
    int realDist = distCalc(radioPackage.latitude, radioPackage.longitude, recievePackage.latitude, recievePackage.longitude);
    lcd.print(realDist);
  }
  
  // and print all our other stuff
  lcd.home();
  lcd.print("LNG");
  lcd.setCursor(8,0);
  lcd.print("LAT");
  lcd.setCursor(0,1);
  lcd.print("SPD");
  lcd.setCursor(5,1);
  lcd.print("HD");
  lcd.setCursor(10,1);
  lcd.print("DST");

  // and do stuff on the serial monitor
    Serial.print(F("Recieved info"));
    
    Serial.print(F("Latitude: "));
    Serial.print(recievePackage.latitude, 6);
    Serial.print(F("  "));

    Serial.print(F("Longitude: "));
    Serial.print(recievePackage.longitude, 6);
    Serial.print(F("  "));

    Serial.print(F("Speed: "));
    Serial.print(recievePackage.speed);
    Serial.print(F(" mph  "));

    double realestCourse = recievePackage.course/100.0;
    Serial.print(F("Course: "));
    Serial.print(realestCourse);
    Serial.print(F(" degrees  "));

    double realestAltitude = recievePackage.altitude/100.0;
    Serial.print(F("Altitude: "));
    Serial.print(realestAltitude);
    Serial.println(F(" meters  "));
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


    Serial.print(F("Calculated dist is:  "));
    Serial.println(distance*1000.0);
    return distance*1000.0;
}

