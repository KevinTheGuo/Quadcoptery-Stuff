/* This sketch involves a hand-controlled controller, transmitting instructions to a second arduino to control
 *  the quadcopter. Pitch, roll, and heading are all determined by the orientation of the hand. A slider determines throttle.
 *  VERSION 2.5 IS FOR APM COMPATIBILITY
 */
/* PINOUT:
 *  Analog output for throttle- A0
 *  SDA from MPU- A4
 *  SCL from MPU- A5
 */
// includes for i2c and communication stuff
#include "Wire.h"

// includes for GPS
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// includes for radio communication
#include <SPI.h>
#include "RF24.h"

// includes for sensors
#include "BMP085.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <HMC5883L.h>

// SETTINGS WE MAY NEED TO CHANGE
#define XGyroOffset     -157
#define YGyroOffset      84
#define ZGyroOffset      29
#define ZAccelOffset     411
float declinationAngle = (-3.0 + (11.0 / 60.0)) / (180 / M_PI);   // this is declination angle for UIUC! Change if you are somewhere else.

// const ints for the GPS pin
static const int RXPin = 4, TXPin = 3;

// initialize objects for each of our sensors
BMP085 barometer;
MPU6050 mpu;
HMC5883L compass;

// MPU control and status variables
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// MPU orientation and motion   variables
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// global variables for the mpu
float currYaw, currPitch, currRoll, prevYaw, prevPitch, prevRoll;   // holds orientation

// global variable for the magnetometer
float heading, prevHeading;          //  holds current and prev heading
int antiGlitch = 10;     // check this flag to see how long we can keep throwing out bad values

// our timer variable
unsigned long currentMillis = 0; 
unsigned long previousMillis = 0;

// the Radio object
RF24 radio(7,8);

// Declare addresses for radio transmission
byte addresses[][6] = {"trans","recev"};

// our struct to send stuff
struct dataPackage{
  int throttle;    
  int roll;
  int pitch;
  int yaw;
  int aux1;
  int info;   // NOTE: aux2 currently not being forwarded to multiwii. possible use as another control signal?
  int checksum;
};
struct dataPackage radioPackage;

// random variables and stuff
int serialIn;   // creating variable to hold input signals

void setup() {  
  Serial.begin(38400);
  Serial.println(F("Welcome to the the Quad Controller"));

  // join I2C bus 
  Wire.begin();
  Wire.setClock(4200);   // blazeit!
  
  Serial.println(F("------------------------------------------------------------------------------------------------"));
  Serial.println(F("Rotate and tilt your hands to control the quadcopter! Use the slider to control the throttle "));
  Serial.println(F("Use keys 3/4 to control aux1. Use keys 5/6 to control yaw "));
  Serial.println(F("To enter LAND mode, press 'l'"));
  Serial.println(F("To enter EMERGENCY SHUTDOWN mode, just press 'e'"));
  Serial.println(F("------------------------------------------------------------------------------------------------"));

  // initialize sensors!
  mpu.dmpInitialize();    // initialize our mpu with the dmp!
  mpu.setXGyroOffset(XGyroOffset);     // set offsets from test
  mpu.setYGyroOffset(YGyroOffset);
  mpu.setZGyroOffset(ZGyroOffset);
  mpu.setZAccelOffset(ZAccelOffset);
  mpu.setDMPEnabled(true);    // enable our DMP!! this takes load off our arduino and gives nice results!
  packetSize = mpu.dmpGetFIFOPacketSize();    // get DMP packet size for later comparison

  compass.begin();    // initialize our compass!
  compass.setRange(HMC5883L_RANGE_1_3GA);  // Set measurement range
  compass.setMeasurementMode(HMC5883L_CONTINOUS);  // Set measurement mode
  compass.setDataRate(HMC5883L_DATARATE_30HZ);  // Set data rate
  compass.setSamples(HMC5883L_SAMPLES_8);  // Set number of samples averaged
  compass.setOffset(99, 153);   // Set calibration offset. See HMC5883L_calibration.ino   Format is (x,y)

  barometer.bmp085Calibration();        // calibrate our pressure sensor
  
  // set our radio stuff
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_MAX); // we might need to do this or not

  // open radio pipes
  radio.openWritingPipe(addresses[1]);
//  radio.openReadingPipe(1,addresses[0]);
  radio.stopListening();

  // and now setup our struct with default values. The range is 125-250, and the center is 187
  radioPackage.throttle = 0;
  radioPackage.roll = 0;
  radioPackage.pitch = 0;
  radioPackage.yaw = 0;
  radioPackage.aux1 = 50;
  radioPackage.info = 0;
  radioPackage.checksum = radioPackage.throttle + radioPackage.roll + radioPackage.pitch + radioPackage.yaw + radioPackage.aux1 + radioPackage.info;  // literally sum

  // and initialize the slide potentiometer! at A0
  pinMode(A0, INPUT);
}

void loop() 
{
//DBUG  Serial.print(F("(1)"));
    // first save our time
  currentMillis = millis();

  // grab barometer information
  float temperature = barometer.bmp085GetTemperature(); //MUST be called first?
  float pressure = barometer.bmp085GetPressure();
  float altitude = barometer.calcAltitude(pressure);

  // grab compass information
  Vector mag = compass.readNormalize();
  prevHeading = heading;
  heading = processHeading(mag);
//DBUG  Serial.print(F("(2)"));
  // grab MPU information
  fifoCount = mpu.getFIFOCount();     // get current FIFO count
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();        // wait for correct available data length
  mpu.getFIFOBytes(fifoBuffer, packetSize);       // read a packet from FIFO 
  mpu.resetFIFO();    // CLEAR THE BUFFER!!
  fifoCount -= packetSize;      // track FIFO count here in case there is > 1 packet available

  // now we can grab yaw, pitch and roll from the dmp buffer
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpu.dmpGetAccel(&aa, fifoBuffer);

  // and turn them into nice degrees
  // currYaw = (ypr[0] * 180/M_PI);    // we have this information, but we discard it because the magnetometer is much nicer
  currPitch = (ypr[1] * 180/M_PI);    // flip pitch and roll because we upside down!
  currRoll = -(ypr[2] * 180/M_PI);    
//DBUG  Serial.print(F("(3)"));
  // check for serial input
  if (Serial.available() > 0)
  {
    serialIn = Serial.read();
    if ((serialIn == '3') && (radioPackage.aux1 > 0))           // 51 THIS IS "3" 
    {
      radioPackage.aux1 = radioPackage.aux1 - 10;
      Serial.print(F("AUX1 down to "));      
      Serial.println(radioPackage.aux1);
    }
    else if ((serialIn == '4') && (radioPackage.aux1 < 100))           // 52 THIS IS "4"
    {
      radioPackage.aux1 = radioPackage.aux1 + 10;
      Serial.print(F("AUX1 up to "));         
      Serial.println(radioPackage.aux1);
    }
    else if ((serialIn == '5') && (radioPackage.yaw > -70))        // YAW LEFT
    {
      radioPackage.yaw = radioPackage.yaw - 10;
      Serial.print(F("YAW down to "));      
      Serial.println(radioPackage.yaw);
    }
    else if ((serialIn == '6') && (radioPackage.yaw < 70))         // YaW RIGHT
    {
      radioPackage.yaw = radioPackage.yaw + 10;
      Serial.print(F("YAW up to "));         
      Serial.println(radioPackage.yaw);  
    }
    else if (serialIn == 'e')            
    {
      Serial.print(F("EMERGENCY STOP initiated. . .  "));
      radioPackage.aux1 =  10;
      radioPackage.throttle = radioPackage.throttle - 20;
      delay(500);
      radioPackage.throttle = 0;
      radio.write(&radioPackage, sizeof(radioPackage));
      Serial.println(F("EMERGENCY STOP finished. Restart device to gain control again."));
      while(1);   // just freeze.
    }
    else if (serialIn > 42) // ignore newlines and stuff
    {
      Serial.print(F("INVALID COMMAND"));
    }
  }
//  Serial.print(F("(4)"));
  // now we can translate our orientation to radio signals! do some quick maths
  int throttleIn = analogRead(A0);  // read from analog 0
  if (throttleIn > 900)             // At high levels, it will taper out, so alleviate that a bit
    radioPackage.throttle = analogRead(A0)/8 - 37;      
  else
    radioPackage.throttle = analogRead(A0)/12;
  radioPackage.roll = currRoll/2;
  radioPackage.pitch = currPitch/2;
  
//DBUG  Serial.print(F("(5)"));
  
  if((currentMillis - previousMillis) > 100)   // wait .1 seconds between displaying/transmitting
  {
    Serial.print(F("Status: "));
    if(radioPackage.aux1 <= 10)
      Serial.print(F("STABILIZE"));
    else if(radioPackage.aux1 < 90)
      Serial.print(F("ALT HOLD"));
    else
      Serial.print(F("LAND"));
      
    Serial.print(F("  Throttle: "));
    Serial.print(radioPackage.throttle);    
    Serial.print(F("  Roll: "));
    Serial.print(radioPackage.roll);
    Serial.print(F("  Pitch: "));
    Serial.print(radioPackage.pitch);
    Serial.print(F("  Yaw: "));
    Serial.print(radioPackage.yaw);

    if ((radioPackage.yaw >= 60) && (radioPackage.throttle == 0))
      Serial.print(F("--  ARMING..."));
    Serial.println("");
    
//DBUG    Serial.print(F("(6)")); 
    radioPackage.checksum = radioPackage.throttle + radioPackage.roll + radioPackage.pitch + radioPackage.yaw + radioPackage.aux1 + radioPackage.info;
    radio.write(&radioPackage, sizeof(radioPackage));
    previousMillis = currentMillis;
//DBUG    Serial.print(F("(7)"));
  }
}

