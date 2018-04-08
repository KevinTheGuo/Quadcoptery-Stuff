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

int outputStatus = 1;
int analogOutput = 1000;

void setup() {
  Serial.begin(38400);
  Serial.println(F("Welcome to the the ESC Calibrator!"));  

  pinMode(ESC1, OUTPUT);   // sets the pin as output
  pinMode(ESC2, OUTPUT);
  pinMode(ESC3, OUTPUT);
  pinMode(ESC4, OUTPUT);
}

void loop() 
{
  analogOutput = analogRead(A0);

  Serial.println(analogOutput);

  analogWrite(ESC1, analogOutput);
  analogWrite(ESC1, analogOutput);
  analogWrite(ESC1, analogOutput);
  analogWrite(ESC1, analogOutput);
}


