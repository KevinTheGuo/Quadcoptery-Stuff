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

// includes for 'servo', AKA software PWM, to control the flight controller
#include <Servo.h>

/*
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
 *  9- bottom right (green)
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
// all of the above are OBSOLETE
// aux1 increase "4"
// aux1 decrease "3"
// aux2 increase "5"
// aux2 decrease "6"

Servo throttleIn;  // create servo object to control signals
Servo rollIn;
Servo pitchIn;
Servo yawIn;
Servo aux1In;
Servo aux2In;

// const ints for the GPS pin
// static const int RXPin = 4, TXPin = 3;
// static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
// TinyGPSPlus gps;

// The serial connection to the GPS device
// SoftwareSerial gpss(RXPin, TXPin);

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
unsigned long currentMillis = 0; 
unsigned long previousMillis = 0;
int yawControl;

// the Radio object
RF24 radio(7,8);

// Declare addresses for radio transmission
byte addresses[][6] = {"trans","recev"};

// our struct
struct dataPackage{
  int throttle;    
  int roll;
  int pitch;
  int heading;
  int aux1;
  int aux2;
};
struct dataPackage radioPackage;

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
//  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);
  radio.startListening();

  // set up our GPS. Temporarily paused until we move all controls to hw PWM
 // gpss.begin(GPSBaud);

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
  
  // and now setup our struct
  radioPackage.throttle = 48;
  radioPackage.roll = 93;
  radioPackage.pitch = 93;
  radioPackage.heading = 93;
  radioPackage.aux1 = 93;
  radioPackage.aux2 = 93;

  throttleIn.attach(5);  // attach our stuff!
  rollIn.attach(6);
  pitchIn.attach(9);
  yawIn.attach(10);
  aux1In.attach(14);
  aux2In.attach(15);
}

void loop() 
{
/*  while (gpss.available() > 0)
  {
    if (gps.encode(gpss.read()))
    {
      Serial.println(F("We encoded our gps stuff"));
    }
  }
  */

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
  
  if(radio.available())
  {
    //Serial.println(F("We got something!"));
    if(radio.available())
    {
      radio.read(&radioPackage, sizeof(radioPackage));
    }

    // calculate yaw change to match desired heading
    desiredHeading = radioPackage.heading;    // we now have a new desired heading!
    int headingDiff = getHeadingDiff(currHeading, radioPackage.heading);
    if(abs(headingDiff) > 10)  // if difference > 10, then yaw!
      yawControl = headingDiff/6 + 93;
    else
      yawControl = 93;    // otherwise, don't yaw

    // write our stuff in if we have a change!
    throttleIn.write(radioPackage.throttle);
    rollIn.write(radioPackage.roll);
    pitchIn.write(radioPackage.pitch);
    yawIn.write(yawControl);      
    aux1In.write(radioPackage.aux1);
    aux2In.write(radioPackage.aux2);   
  }

  currentMillis = millis();
  if((currentMillis - previousMillis) > 100)   // wait .1 seconds between displaying
  {
    Serial.print(F("throttle: "));
    Serial.print(radioPackage.throttle);    
    Serial.print(F("  roll: "));
    Serial.print(radioPackage.roll);
    Serial.print(F("  pitch: "));
    Serial.print(radioPackage.pitch);
    Serial.print(F("  desired heading: "));
    Serial.print(desiredHeading);
    Serial.print(F("  current heading: "));
    Serial.print(currHeading);
    Serial.print(F("  heading diff: "));
    Serial.print(getHeadingDiff(currHeading, desiredHeading));
    Serial.print(F("  yaw: "));
    Serial.print(yawControl);
    Serial.print(F("  aux1: "));
    Serial.print(radioPackage.aux1);   
    Serial.print(F("  aux2: "));
    Serial.println(radioPackage.aux2);     
    previousMillis = currentMillis;
  }
}


// tilt compensation, refining, and error scrubbing
float processHeading(Vector mag)
{  
  // convert them to pitch and roll
  float roll;
  float pitch;

  roll = asin(-ypr[2]);   // we've got this info already from the dmp
  pitch = asin(ypr[1]);

  if (roll > 0.6 || roll < -0.6 || pitch > 0.6 || pitch < -0.6)
    return prevHeading;

    // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
  float cosRoll = cos(roll);  
  float sinRoll = sin(roll);  
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);
  
  // Tilt compensation
  float Xh = mag.XAxis * cosPitch + mag.ZAxis * sinPitch;
  float Yh = mag.XAxis * sinRoll * sinPitch + mag.YAxis * cosRoll - mag.ZAxis * sinRoll * cosPitch;
 
  float process_heading = atan2(Yh, Xh);

  // adjust for declination angle
  declinationAngle = (-3.0 + (11.0 / 60.0)) / (180 / M_PI);   // this is declination angle for uiuc!
  process_heading += declinationAngle;

  process_heading = correctAngle(process_heading);    // correct angle for if its above 2pi or less than 0
  process_heading = process_heading * 180/M_PI;     // convert to degrees

  // Fix magnetometer issue with angles to make it more accurate
  float fixedHeadingDegrees;
  if (process_heading >= 1 && process_heading < 240)
  {
    fixedHeadingDegrees = map(process_heading, 0, 239, 0, 179);
  } else
  if (process_heading >= 240)
  {
    fixedHeadingDegrees = map(process_heading, 240, 360, 180, 360);
  }

  process_heading = fixedHeadingDegrees;

  // check for unusual values
  int testHeading = process_heading;
  int oldHeading = prevHeading;
  int angleChange = ((testHeading - oldHeading + 180 + 360)%360) - 180;   // calculate difference, but keep in mind degrees!
  if((angleChange > 60) && (antiGlitch < 5))  // if difference > 50 and we haven't been throwing out values too long..
  {
    process_heading = prevHeading;
    antiGlitch++;
  }
  else
    antiGlitch = 0;

  // more often than not, a heading of 0 is erroneous. If it's 0, just make it previous heading.
  if (process_heading == 0)
    process_heading = prevHeading;
  
  return process_heading;   // return the smoothed, calibrated heading!
}

// Correct angle
float correctAngle(float process_heading)
{
  if (process_heading < 0) { process_heading += 2 * PI; }
  if (process_heading > 2 * PI) { process_heading -= 2 * PI; }

  return process_heading;
}

// Get a signed difference between two headings
int getHeadingDiff(float initial, float dest)
{
  if (initial > 360 || initial < 0 || dest > 360 || dest < 0)
  {
    Serial.print(F("Heading Diff Error!"));
    return 0;
  }

  int signed_diff = dest - initial;
  int abs_diff = abs(dest - initial);

  if (abs_diff <= 180)
    return abs_diff == 180 ? abs_diff : signed_diff;
  else if (dest-initial > 0)
    return abs_diff - 360;
  else
    return 360 - abs_diff;
}

