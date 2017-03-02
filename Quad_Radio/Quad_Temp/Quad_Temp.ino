#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Servo.h>
#include "RF24.h"

/* This sketch involves the receiver (quadcopter) receiving instructions from a second arduino to send signals to control
 *  the quadcopter
 *  
 *  The pinout is like this:
 *  THROTTLE- pin 5 (yellow)
 *  ROLL(aileron)- pin 6 (green)
 *  PITCH(elevator)- pin 9 (blue)
 *  YAW(rudder)- pin 10 (purple)
 *  AUX1- pin 14/A0 (white)
 *  AUX2- pin 15/A1 (grey)
 *  
 *  
 *  MULTIWII ARDUINO CONNECTIONS
 *  from IMU perspective-
 *  3- top left (orange)
 *  10- top right (purple)
 *  11- bottom left (blue)
 *  12- bottom right (green)
 */

// controls
// throttle up "1"
// throttle down "2"
// roll right "e"
// roll left "q"
// pitch forward "w"
// pitch backward "s"
// yaw right "d"
// yaw left "a"
// aux1 increase "4"
// aux1 decrease "3"

Servo throttleIn;  // create servo object to control signals
Servo rollIn;
Servo pitchIn;
Servo yawIn;
Servo aux1In;
//  Servo aux2In;

// const ints for the GPS pin
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// our struct
struct dataPackage{
  int throttle;    
  int roll;
  int pitch;
  int yaw;
  int aux1;
  int aux2;
};
struct dataPackage radioPackage;

void setup() {
  Serial.begin(9600);
 // Serial.println(F("Welcome to the samoyed transceiving unit"));


  // and now setup our struct
  radioPackage.throttle = 140;
  radioPackage.roll = 93;
  radioPackage.pitch = 93;
  radioPackage.yaw = 93;
  radioPackage.aux1 = 93;
  radioPackage.aux2 = 93;

  throttleIn.attach(5);  // attach our stuff!
  rollIn.attach(6);
  pitchIn.attach(9);
  yawIn.attach(10);
  aux1In.attach(14);
//  aux2In.attach(15);
}

void loop() 
{

    Serial.println(F("woo"));
    Serial.print(radioPackage.throttle);
    Serial.print(F("    "));
    Serial.print(radioPackage.roll);
    Serial.print(F("    "));
    Serial.print(radioPackage.pitch);
    Serial.print(F("    "));
    Serial.print(radioPackage.yaw);
    Serial.print(F("    "));
    Serial.println(radioPackage.aux1);

    throttleIn.write(70);
    rollIn.write(120);

}

