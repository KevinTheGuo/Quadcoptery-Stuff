// Program to test Bluetooth Module
// Password for bluetooth connection is '123456'

// Include for RX/TX communication
#include <SoftwareSerial.h>

// PINOUT:
// Pin 10- TXD (RX on Bluetooth Module)
// Pin 11- RX (TXD on Bluetooth Module)

// Instantiate pins for rx/tx communication
// SoftwareSerial BluetoothCommunication(10, 11); // RX, TX

void setup() {
  // Start serial communication using the Bluetooth Module
  Serial.begin(9600);
  Serial.println("Bluetooth Online!");
  pinMode(13,OUTPUT);
}

void loop() {
  if (Serial.available())
  {
    char input = Serial.read();
    Serial.println("We got something!");

    // Blink LED depending on data sent to bluetooth module
    if(input=='1')
    {   
    digitalWrite(13,1);
    Serial.println("Turning on LED");
    }
    else if (input=='0')
    {
      digitalWrite(13,0);
      Serial.println("Turning off LED");
    }
  }
  delay(100);   // wait a lil bit
  //BluetoothCommunication.println("Huskies r SWAG");
  //Serial.println("Nothing yet");
}
