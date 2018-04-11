//// includes for i2c and communication stuff
//#include "Wire.h"
//
//// includes for GPS
//#include <TinyGPS++.h>
//#include <SoftwareSerial.h>
//
//// includes for radio communication
//#include <SPI.h>
//#include "RF24.h"
//
//// includes for sensors
//#include "BMP085.h"
//#include "MPU6050_6Axis_MotionApps20.h"
//#include <HMC5883L.h>
//
//// SETTINGS WE MAY NEED TO CHANGE
//#define XGyroOffset     -157
//#define YGyroOffset      84
//#define ZGyroOffset      29
//#define ZAccelOffset     411
//float declinationAngle = (-3.0 + (11.0 / 60.0)) / (180 / M_PI);   // this is declination angle for UIUC! Change if you are somewhere else.
//
//// const ints for the GPS pin
//static const int RXPin = 4, TXPin = 3;
//
//// initialize objects for each of our sensors
//BMP085 barometer;
//MPU6050 mpu;
//HMC5883L compass;
//
//// MPU control and status variables
//uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
//uint16_t fifoCount;     // count of all bytes currently in FIFO
//uint8_t fifoBuffer[64]; // FIFO storage buffer
//
//// MPU orientation and motion   variables
//Quaternion q;           // [w, x, y, z]         quaternion container
//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorFloat gravity;    // [x, y, z]            gravity vector
//float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//
//// global variables for the mpu
//float currYaw, currPitch, currRoll, prevYaw, prevPitch, prevRoll;   // holds orientation
//
//// global variable for the magnetometer
//float heading, prevHeading;          //  holds current and prev heading
//int antiGlitch = 10;     // check this flag to see how long we can keep throwing out bad values
//
//// our timer variable
//unsigned long currentMillis = 0; 
//unsigned long previousMillis = 0;
//
//// the Radio object
//RF24 radio(7,8);
//
//// Declare addresses for radio transmission
//byte addresses[][6] = {"trans","recev"};
//
//// our struct to send stuff
//struct dataPackage{
//  int throttle;    
//  int roll;
//  int pitch;
//  int heading;
//  int aux1;
//  int info;   // NOTE: aux2 currently not being forwarded to multiwii. possible use as another control signal?
//  int checksum;
//};
//struct dataPackage radioPackage;
//
//// random variables and stuff
//int serialIn;   // creating variable to hold input signals
