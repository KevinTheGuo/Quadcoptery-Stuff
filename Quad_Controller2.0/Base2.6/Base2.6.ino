/* This sketch involves a base station, transmitting instructions a quadcopter
 *  Pitch, roll, and heading are all determined by the orientation an MPU. A slider determines throttle.
 *  VERSION 2.6 IS FOR APM COMPATIBILITY. Automatic AUX and YAW control
 *  Is main writer, but receive acknowledgements containing telemetry information from quadcopter. 
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

// MPU interruption detection
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// global variables for the mpu
float currYaw, currPitch, currRoll, prevYaw, prevPitch, prevRoll;   // holds orientation

// global variable for the magnetometer
float currHeading, prevHeading;          //  holds current and prev heading
int antiGlitch = 5;     // check this flag to see how long we can keep throwing out bad values

// Other Assorted Variables for Controlling the Quadcopter
int headingAtArm = 0;        // mildly temporary variable, set at arm, that ensures the quadcopter stays at same heading as it was when armed.
int isArmed = 0;              // lets quadcopter know if it's armed or not.

// our timer variable
unsigned long previousMillis = 0;

// the Radio object
RF24 radio(7,8);

// Declare addresses for radio transmission. Address[0] is used for quad writing, base reading, and Address[1] used for quad reading, base writing
byte addresses[][6] = {"quadw","basew"};

// Define structs for input and output
struct controlPackage{
  int throttle = 0;    
  int roll = 0;
  int pitch = 0;
  int yaw = 0;
  int aux1 = 0;
  int info = 0;  
  int checksum = 0;
};
struct telemetryPackage{
//  float latitude;      // currently unused
//  float longitude;     // currently unused
  float alt = 0.0;
  int heading = 0;
  int info = 0;   
  int checksum = 0;
};
struct controlPackage controls;
struct telemetryPackage telemetry;

void setup() {  
  Serial.begin(38400);
  Serial.println(F("Welcome to the the Quad Controller"));

  // join I2C bus 
  Wire.begin();
  Wire.setClock(4200);   // blazeit!
  
  Serial.println(F("------------------------------------------------------------------------------------------------"));
  Serial.println(F("Rotate and tilt your hands to control the quadcopter! Use the slider to control the throttle "));
  Serial.println(F("Use keys 3/4 to control aux1. Use keys 5/6 to control yaw "));
  Serial.println(F("To ARM the quadcopter, make sure throttle is at 0, and press 'a'"));
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
  compass.setOffset(0,0);   // Set calibration offset. See HMC5883L_calibration.ino   Format is (x,y)

  barometer.bmp085Calibration();        // calibrate our pressure sensor
  
  // set our radio stuff
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_MAX); // we might need to do this or not
  radio.enableAckPayload();                     // Allow optional ack payloads
  radio.enableDynamicPayloads();                // Ack payloads are dynamic payloads
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1,addresses[0]);
  radio.stopListening();

  // and initialize the slide potentiometer! at A0
  pinMode(A0, INPUT);
}

void loop() 
{
  // grab barometer information
  float temperature = barometer.bmp085GetTemperature(); //MUST be called first?
  float pressure = barometer.bmp085GetPressure();
  float altitude = barometer.calcAltitude(pressure);

  // grab compass information
  Vector mag = compass.readNormalize();
  prevHeading = currHeading;
  currHeading = processHeading(mag);

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

  // check for serial input
  if (Serial.available() > 0)
  {
    int serialIn = Serial.read();
    if ((serialIn == '3') && (controls.aux1 > 0))           // 51 THIS IS "3" 
    {
      controls.aux1 = controls.aux1 - 10;
      Serial.print(F("AUX1 down to "));      
      Serial.println(controls.aux1);
    }
    else if ((serialIn == '4') && (controls.aux1 < 100))           // 52 THIS IS "4"
    {
      controls.aux1 = controls.aux1 + 10;
      Serial.print(F("AUX1 up to "));         
      Serial.println(controls.aux1);
    }
    else if ((serialIn == '5') && (controls.yaw > -80))        // YAW LEFT
    {
      controls.yaw = controls.yaw - 10;
      Serial.print(F("YAW down to "));      
      Serial.println(controls.yaw);
    }
    else if ((serialIn == '6') && (controls.yaw < 80))         // YaW RIGHT
    {
      controls.yaw = controls.yaw + 10;
      Serial.print(F("YAW up to "));         
      Serial.println(controls.yaw);  
    }
    else if ((serialIn == 'a') && (controls.throttle == 0))      // Save current heading, and arm the quadcopter  
    {
      controls.info = 1;
      controls.checksum = controls.throttle + controls.roll + controls.pitch + controls.yaw + controls.aux1 + controls.info;
      radio.write(&controls, sizeof(controls));
      controls.info = 0;
      headingAtArm = telemetry.heading;
      previousMillis = millis();
      isArmed = 1;
      Serial.println(F("ARMING QUADCOPTER!!"));
    }
    else if ((serialIn == 'd') && (controls.throttle < 20))      // disarm the quadcopter
    {
      controls.info = -1;
      controls.checksum = controls.throttle + controls.roll + controls.pitch + controls.yaw + controls.aux1 + controls.info;
      radio.write(&controls, sizeof(controls));
      controls.info = 0;
      previousMillis = millis();
      isArmed = 0;
      controls.yaw = 0;
      Serial.println(F("DISARMING QUADCOPTER..."));
    }
    else if (serialIn == 'e')            
    {
      Serial.print(F("EMERGENCY STOP initiated. . .  "));
      controls.aux1 =  10;
      controls.throttle = controls.throttle - 20;
      delay(500);
      controls.throttle = 0;
      radio.write(&controls, sizeof(controls));
      Serial.println(F("EMERGENCY STOP finished. Restart device to gain control again."));
      while(1);   // just freeze.
    }
    else if (serialIn > 42) // ignore newlines and stuff
    {
      Serial.print(F("INVALID COMMAND- "));
    }
  }

  // now we can translate our orientation to radio signals! do some quick maths
  int throttleIn = analogRead(A0);  // read from analog 0
  if (throttleIn > 900)             // At high levels, it will taper out, so alleviate that a bit
    controls.throttle = throttleIn/8 - 37;      
  else
    controls.throttle = throttleIn/12;
//  controls.roll = currRoll/2;
//  controls.pitch = currPitch/2;
  controls.roll = 0;
  controls.pitch = 0;
  
  if((millis() - previousMillis) > 100)   // wait .1 seconds between displaying/transmitting
  {
    Serial.print(F("Mode: "));
    if(controls.aux1 <= 10)
      Serial.print(F("STABILIZE"));
    else if(controls.aux1 < 90)
      Serial.print(F("ALT HOLD"));
    else
      Serial.print(F("LAND"));
      
    Serial.print(F("  Throttle: "));
    Serial.print(controls.throttle);    
    Serial.print(F("  Roll: "));
    Serial.print(controls.roll);
    Serial.print(F("  Pitch: "));
    Serial.print(controls.pitch);
    Serial.print(F("  Quad Heading: "));
    Serial.print(telemetry.heading);
    Serial.print(F("  Arm Heading: "));
    Serial.print(headingAtArm);
    Serial.print(F("  Yaw: "));
    Serial.print(controls.yaw);
    Serial.print(F("  Quad Alt: "));  
    Serial.print(telemetry.alt);

    if ((controls.yaw >= 60) && (controls.throttle == 0))
      Serial.print(F(" -  ARMING!!!"));
    Serial.println("");
    
    controls.checksum = controls.throttle + controls.roll + controls.pitch + controls.yaw + controls.aux1 + controls.info;
    radio.write(&controls, sizeof(controls));
    previousMillis = millis();

    while (radio.available())    // If an ack with payload was received
    {                      
       radio.read(&telemetry, sizeof(telemetry));                  // Read it, and display the response time
       if (telemetry.checksum != int(telemetry.alt + telemetry.heading + telemetry.info))
         Serial.println(F("  Warning: Telemetry Checksum failed"));
       else if (isArmed)
       {
         int headingDiff = getHeadingDiff(headingAtArm, telemetry.heading);  // we now have a new desired heading! For now, set the comparison header to a constant.
//         controls.yaw = headingDiff/2;
          controls.yaw = 0;
       }
    }
  }
}

