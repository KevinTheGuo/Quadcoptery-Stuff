/* This sketch deals with a receiever, receiving instructions from a hand-controlled controller, which it will try
 *  to match the orientation of. pitch, roll, and heading are all taken into account. altitude coming soon!!
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
#define XGyroOffset      67
#define YGyroOffset     -11
#define ZGyroOffset      49
#define ZAccelOffset     4594
float declinationAngle = (-3.0 + (11.0 / 60.0)) / (180 / M_PI);   // this is declination angle for UIUC! Change if you are somewhere else.


/*  PINOUT
 *  
 *  Multiwii:
 *  THROTTLE- pin 5 (yellow)
 *  ROLL(aileron)- pin 6 (green)
 *  PITCH(elevator)- pin 9 (blue)
 *  YAW(rudder)- pin 10 (purple)
 *  AUX1- pin 3 (white)
 *  
 *  Radio:
 *  CE- pin 7 (orange)
 *  CSN- pin 8 (yellow)
 *  SCK- pin 13 (green)
 *  MOSI- pin 11 (blue)
 *  MISO- pin 12 (purple)
 *  
 */

 // Define pin variables
#define THROTTLE_PIN 5
#define ROLL_PIN     6
#define PITCH_PIN    9
#define YAW_PIN      10
#define AUX1_PIN     3

// const ints for the GPS pin
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial gpss(RXPin, TXPin);

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
float currHeading, prevHeading;          //  holds current and prev heading
int antiGlitch = 5;     // check this flag to see how long we can keep throwing out bad values
int desiredHeading;    // variables to control the quadcopter

// TEMPORARY
unsigned long previousMillis = 0;
int yawControl;

// Heartbeat variable
unsigned long lastMessageMillis = 0;


// the Radio object
RF24 radio(7,8);

// Declare addresses for radio transmission
byte addresses[][6] = {"trans","recev"};

// our struct
struct inputPackage{
  int throttle;    
  int roll;
  int pitch;
  int heading;
  int aux1;
  int info;   // NOTE: aux2 currently not being forwarded to multiwii. possible use as another control signal?
  int checksum;
};

// our struct
struct outputPackage{
  int throttle;    
  int roll;
  int pitch;
  int heading;
  int aux1;
  int info;   // NOTE: aux2 currently not being forwarded to multiwii. possible use as another control signal?
  int checksum;
};


struct inputPackage radioPackageIn;
struct outputPackage radioPackageOut;

void setup() {
  Serial.begin(38400);
  Serial.println(F("Welcome to the the Quad Receiver!"));

  // join I2C bus 
  Wire.begin();
  Wire.setClock(4200);   // blazeit!
  
  // set our radio stuff
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_MAX); // we might need to do this or not

  // open radio pipes
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);
  radio.startListening();

  // set up our GPS. Temporarily paused until we move all controls to hw PWM
  gpss.begin(GPSBaud);

  // initialize sensors!
  mpu.dmpInitialize();    // initialize our mpu with the dmp!   DONT NEED MPU RIGHT NOW, PROBABLY NEVER WILL
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
  compass.setOffset(-20, -108);   // Set calibration offset. See HMC5883L_calibration.ino. Format is x,y

  barometer.bmp085Calibration();        // calibrate our pressure sensor
  
  // and now setup our struct with default values. The range is 125-250
  radioPackageIn.throttle = 0;
  radioPackageIn.roll = 187;
  radioPackageIn.pitch = 187;
  radioPackageIn.heading = 0;
  radioPackageIn.aux1 = 187;
  radioPackageIn.info = 0;
  radioPackageIn.checksum = radioPackageIn.throttle + radioPackageIn.roll + radioPackageIn.pitch + radioPackageIn.heading + radioPackageIn.aux1 + radioPackageIn.info;
  yawControl = 187;

  // Initialize our PWM outputs to the MultiWii
  pinMode(THROTTLE_PIN, OUTPUT);   // sets the pin as output
  pinMode(ROLL_PIN, OUTPUT);
  pinMode(PITCH_PIN, OUTPUT);
  pinMode(YAW_PIN, OUTPUT);
  pinMode(AUX1_PIN, OUTPUT);

  // Let's see how well we do with a modified Timer0. This fixes the frequency problem with pins 5 and 6, but modifies millis
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00); 

  lastMessageMillis = millis();   // initialize our heartbeat variable to current time
}

void loop() 
{
  while (gpss.available() > 0)
  {
    if (gps.encode(gpss.read()))
    {
      Serial.println(F("We encoded our gps stuff"));
    }
  }

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
  
  while(radio.available())
  {
    lastMessageMillis = millis();   // update our timestamp for last message received
    
    //Serial.println(F("We got something!"));
    radio.read(&radioPackageIn, sizeof(radioPackageIn));

    // Verify that the message is valid by literally checking the sum (hahaha)
    if (radioPackageIn.checksum != radioPackageIn.throttle + radioPackageIn.roll + radioPackageIn.pitch + radioPackageIn.heading + radioPackageIn.aux1 + radioPackageIn.info)
    {
      Serial.println(F("ERR: CheXum Invalid!"));
      break;
    }

    // calculate yaw change to match desired heading
    desiredHeading = radioPackageIn.heading;    // we now have a new desired heading!
    int headingDiff = getHeadingDiff(currHeading, radioPackageIn.heading);
    if(abs(headingDiff) > 10)  // if difference > 10, then yaw!
      yawControl = headingDiff/6 + 187;
    else
      yawControl = 187;    // otherwise, don't yaw

    // write our stuff in if we have a change!
    analogWrite(THROTTLE_PIN, radioPackageIn.throttle);
    analogWrite(ROLL_PIN, radioPackageIn.roll);
    analogWrite(PITCH_PIN, radioPackageIn.pitch);
    analogWrite(YAW_PIN, yawControl);
    analogWrite(AUX1_PIN, radioPackageIn.aux1);
  }

  // If we've lost connection, stop moving. 
  if (millis() - lastMessageMillis > 500)    // check if it's more than half a second since we last received a radio message
  {
    analogWrite(ROLL_PIN, 187);   // zero out roll, pitch, and yaw, but keep all other signals the same
    analogWrite(PITCH_PIN, 187);
    analogWrite(YAW_PIN, 187);
     
    Serial.println(F("ERR: Lost connection!"));
  }

  if((millis() - previousMillis) > 100)   // wait .1 seconds between displaying
  {
    Serial.print(F("throttle: "));
    Serial.print(radioPackageIn.throttle);    
    Serial.print(F("  roll: "));
    Serial.print(radioPackageIn.roll);
    Serial.print(F("  pitch: "));
    Serial.print(radioPackageIn.pitch);
    Serial.print(F("  desired heading: "));
    Serial.print(desiredHeading);
    Serial.print(F("  current heading: "));
    Serial.print(currHeading);
    Serial.print(F("  heading diff: "));
    Serial.print(getHeadingDiff(currHeading, desiredHeading));
    Serial.print(F("  yaw: "));
    Serial.print(yawControl);
    Serial.print(F("  aux1: "));
    Serial.print(radioPackageIn.aux1);   
    Serial.print(F("  info: "));
    Serial.println(radioPackageIn.info);     
    previousMillis = millis();
  }
}


