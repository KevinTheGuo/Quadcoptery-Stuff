#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include "RF24.h"

/* This sketch involves the transmitter (ground station) transmitting instructions to a second arduino to control
 *  the quadcopter
 */

// const ints for the GPS pin
static const int RXPin = 4, TXPin = 3;

// the Radio object
RF24 radio(7,8);

// husky is for GPS writing, sammy is for laptop writing
byte addresses[][6] = {"husky","sammy"};

// our struct to send stuff
struct dataPackage{
  int throttle;    
  int roll;
  int pitch;
  int yaw;
  int aux1;
  int aux2;
};
struct dataPackage radioPackage;

// random variables and stuff
int serialIn;   // creating variable to hold input signals
int stableMode; // mode to hold whether in stable mode or not
unsigned long previousMillis = 0;   // holds previous time we stabilized
int lastInput;    // variable to hold last input





void setup() {
  Serial.begin(9600);
  Serial.println(F("Welcome to the the Quad Transmitter"));
  Serial.println(F("--------------------------------------------------------------------------------"));
  Serial.println(F("Throttle up-2, Throttle down-1, Aux1 up-4, Aux1 down-3"));
  Serial.println(F("High Throttle Mode-0, Low Throttle Mode-9"));
  Serial.println(F("Pitch down-w, Pitch up-s, Roll left-q, Roll right-e, Yaw left-a, Yaw right-d"));
  Serial.println(F("And enter '$' to switch to and from auto-stabilization mode"));
  Serial.println(F("To enter EMERGENCY SHUTDOWN mode, just mush any other random key"));
  Serial.println(F("--------------------------------------------------------------------------------"));
  Serial.println(F("Currently in auto-stabilization mode"));

  stableMode = 1;
  
  // set our radio stuff
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_MAX); // we might need to do this or not

  // open radio pipes
  radio.openWritingPipe(addresses[1]);
//  radio.openReadingPipe(1,addresses[0]);
  radio.stopListening();

  // and now setup our struct
  radioPackage.throttle = 48;
  radioPackage.roll = 93;
  radioPackage.pitch = 93;
  radioPackage.yaw = 93;
  radioPackage.aux1 = 95;
  radioPackage.aux2 = 95;

}

void loop() 
{
  if (Serial.available() > 0)
  {
    serialIn = Serial.read();
    if (serialIn == 49)             // THIS IS "1"
    {
      radioPackage.throttle = radioPackage.throttle - 2;
  //      radioPackage.throttle = 40;        // ESC PROGRAMMING MODE
      if (lastInput == 49)
      {
        Serial.print(F("->"));
      }
      else
      {
        Serial.println(F(" "));
        Serial.print(F("throttle down to "));        
      }
      Serial.print(radioPackage.throttle);
    }
    else if (serialIn == 50)             // THIS IS "2"
    {
      radioPackage.throttle =  radioPackage.throttle + 2;     
 //       radioPackage.throttle = 160;         // ESC PROGRAMMING MODE
      if (lastInput == 50)
      {
        Serial.print(F("->"));
      }
      else
      {
        Serial.println(F(" "));
        Serial.print(F("throttle up to "));        
      }
      Serial.print(radioPackage.throttle);
    }
    else if (serialIn == 51)             // THIS IS "3"
    {
      radioPackage.aux1 = radioPackage.aux1 - 5;
      if (lastInput == 51)
      {
        Serial.print(F("->"));
      }
      else
      {
        Serial.println(F(" "));
        Serial.print(F("aux1 down to "));      
      }
      Serial.print(radioPackage.aux1);
    }
    else if (serialIn == 52)            // THIS IS "4"
    {
      radioPackage.aux1 = radioPackage.aux1 + 5;
      if (lastInput == 52)
      {
        Serial.print(F("->"));
      }
      else
      {
        Serial.println(F(" "));
        Serial.print(F("aux1 up to "));      
      }      
      Serial.print(radioPackage.aux1);
    }
    else if (serialIn == 113)            // THIS IS "q"
    {
      radioPackage.roll = radioPackage.roll - 10;
      if (lastInput == 113)
      {
        Serial.print(F("->"));
      }
      else
      {
        Serial.println(F(" "));
        Serial.print(F("roll left to "));   
      }      
      Serial.print(radioPackage.roll);
    }
    else if (serialIn == 101)            // THIS IS "e"
    {
      radioPackage.roll = radioPackage.roll + 10;
      if (lastInput == 101)
      {
        Serial.print(F("->"));
      }
      else
      {
        Serial.println(F(" "));
        Serial.print(F("roll right to "));   
      }     
      Serial.print(radioPackage.roll);
    }
    else if (serialIn == 119)            // THIS IS "w"
    {
      radioPackage.pitch = radioPackage.pitch - 10;
      if (lastInput == 119)
      {
        Serial.print(F("->"));
      }
      else
      {
        Serial.println(F(" "));
        Serial.print(F("pitch down to "));  
      }           
      Serial.print(radioPackage.pitch);
    }
    else if (serialIn == 115)            // THIS IS "s"
    {
      radioPackage.pitch = radioPackage.pitch + 10;
      if (lastInput == 115)
      {
        Serial.print(F("->"));
      }
      else
      {
        Serial.println(F(" "));
        Serial.print(F("pitch up to "));  
      }  
      Serial.print(radioPackage.pitch);
    }
    else if (serialIn == 97)            // THIS IS "a"
    {
      radioPackage.yaw = radioPackage.yaw - 10;
      if (lastInput == 97)
      {
        Serial.print(F("->"));
      }
      else
      {
        Serial.println(F(" "));
        Serial.print(F("yaw left to "));
      } 
      Serial.print(radioPackage.yaw);
    }
    else if (serialIn == 100)            // THIS IS "d"
    {
      radioPackage.yaw = radioPackage.yaw + 10;
      if (lastInput == 100)
      {
        Serial.print(F("->"));
      }
      else
      {
        Serial.println(F(" "));
        Serial.print(F("yaw right to "));
      } 
      Serial.print(radioPackage.yaw);
    }
    else if (serialIn == 36)
    {
      if(stableMode)
      {
        Serial.println(F("Stable mode deactivated"));
        stableMode = 0;
      }
      else
      {
        Serial.println(F("Stable mode activated"));
        stableMode = 1;        
      }
    }
    else if (serialIn == 57)
    {
        Serial.println(F(" "));
        Serial.print(F("Low Throttle Mode-> 60"));     
        radioPackage.throttle = 60;    
    }
    else if (serialIn == 48)
    {
        Serial.println(F(" "));
        Serial.print(F("High Throttle Mode-> 150"));       
        radioPackage.throttle = 150;
    }
    else
    {
      Serial.print(F("EMERGENCY STOP initiated---- "));
      radioPackage.throttle = radioPackage.throttle - 60;
      delay(500);
      radioPackage.aux1 =  48;
      radioPackage.throttle = 48;
      Serial.println(F("EMERGENCY STOP finished"));
    }
  }

  // save our current input for next round
  lastInput = serialIn;

  // now we do our stabilizing
  unsigned long currentMillis = millis();
  
  if ((stableMode) && (currentMillis - previousMillis >= 10)) // stabilize once every 20th of a second
  {
    previousMillis = currentMillis;
    
    if(radioPackage.roll > 93)
      radioPackage.roll--;
    else if(radioPackage.roll < 93)
      radioPackage.roll++;

    if(radioPackage.pitch > 93)
      radioPackage.pitch--;
    else if(radioPackage.pitch < 93)
      radioPackage.pitch++;

    if(radioPackage.yaw > 93)
      radioPackage.yaw--;
    else if(radioPackage.yaw < 93)
      radioPackage.yaw++;
  }

  // now we send out info!
  // Serial.println(F("Sending our info"));
   
  // now we are constantly sending our stuff
   if (!radio.write(&radioPackage, sizeof(radioPackage)))
   {
     // Serial.print(F("oopsies"));
   }
}
