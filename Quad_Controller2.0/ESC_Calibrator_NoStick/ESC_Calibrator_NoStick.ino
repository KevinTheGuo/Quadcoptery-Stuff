/* This sketch helps us calibrate 4 ESCs simultaneously
 */
 
// includes for 'servo', AKA software PWM, to control the flight controller
#include <Servo.h>

/*
 *  The pinout is like this:
 *  ESC1 - Pin 3
 *  ESC2 - Pin 5
 *  ESC3 - Pin 6
 *  ESC4 - Pin 9
 *  Analog output for throttle- A0
 */

int ESC1 = 3;
int ESC2 = 5;
int ESC3 = 6;
int ESC4 = 9;

int currOutput = 1;

void setup() {
  Serial.begin(38400);
  Serial.println(F("Welcome to the the ESC Calibrator!"));  

  pinMode(ESC1, OUTPUT);   // sets the pin as output
  pinMode(ESC2, OUTPUT);
  pinMode(ESC3, OUTPUT);
  pinMode(ESC4, OUTPUT);

  // initialize our analogOutput to 1010
  analogWrite(ESC1, 1010);
  analogWrite(ESC2, 1010);   
  analogWrite(ESC3, 1010);
  analogWrite(ESC4, 1010);     
}

void loop() 
{
  if (Serial.available() > 0)
  {
    char serialIn = Serial.read();
    if (serialIn == '1')            
    {
      if (currOutput != 1)
      {
        currOutput = 1;
        Serial.println("Moving analogOutput up from 100 to 1010");
        for(int analogOutput=100; analogOutput<=1010; analogOutput+= 5)
        {
          // write our analogInput to all the ESC's!
          analogWrite(ESC1, analogOutput);
          analogWrite(ESC2, analogOutput);   
          analogWrite(ESC3, analogOutput);
          analogWrite(ESC4, analogOutput);     
          Serial.println(analogOutput);
        }
      }
    }
    else if (serialIn == '0')         
    {
      if (currOutput != 0)
      {
        currOutput = 0;
        Serial.println("Moving analogOutput down from 1010 to 100");
        for(int analogOutput=1010; analogOutput>=100; analogOutput-= 5)
        {
          // write our analogInput to all the ESC's!
          analogWrite(ESC1, analogOutput);
          analogWrite(ESC2, analogOutput);   
          analogWrite(ESC3, analogOutput);
          analogWrite(ESC4, analogOutput);     
          Serial.println(analogOutput);
        }
      }
    }
  }
}



