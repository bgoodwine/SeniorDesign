// Include the Wire library for I2C
#include <Wire.h>
#include "Arduino.h"

// LED on pin 13
const int ledPin = 4;

// Function that executes whenever data is received from master
void receiveEvent(int howMany) {
  while (Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    digitalWrite(ledPin, c);
  }
}
void loop() {
  delay(100);
}

void setup() {
  // Join I2C bus as slave with address 8
  uint8_t addr = 0x08;
  // slave addr = 0x08
  // Connect pin 22 on ESP32 to GPIO2 on Pi (SDA)
  // Connect pin 21 on ESP32 to GPIO3 on Pi (SCL)

  Wire.begin(addr, 22, 21); // uint8_t addr, int sda, int scl
  // add to platformio.ini:
  // monitor_speed = 115200
  Serial.begin(115200);
  Serial.println("Joined bus...");
  
  // Call receiveEvent when data received                
  Wire.onReceive(receiveEvent);
  
  // Setup pin 13 as output and turn LED off
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
}
