// Include the Wire library for I2C
#include <Wire.h>
#include "Arduino.h"

// LED on pin 13
const int ledPin = 4;

// Function that executes whenever data is received from master
void receiveEvent(int bytes_to_read) {
  //while (Wire.available()) { // loop through all but the last
  /*char c = 'h';
  while (c != '\0') {
    c = Wire.read();
    Serial.print(c);
  }
  Serial.println("");*/
  int num_bytes = 32;
  Serial.println("Reading...");
  byte buf[num_bytes];
  char msg[num_bytes];
  int msg_len = 0;
  Wire.readBytes(buf, num_bytes);
  Serial.print("Bytes received: ");
  for (int i = 0; i < num_bytes; i++) {
    char hexCar[2];
    sprintf(hexCar, "%02X", buf[i]);
    char c = (char)buf[i];
    if (c != '0' && i != 0) {
      msg[i-1] = c;
      //Serial.print(c);
      msg_len++;
    } else {
      msg[i] = '\0';
    }
    Serial.print(hexCar);
  }

  Serial.println("");
  Serial.print("String recieved: ");
  String m = msg;
  Serial.println(m);
  /*byte buf[bytes_to_read];
  Wire.readBytes(buf, bytes_to_read);
  Serial.print("Received size ");
  Serial.print(bytes_to_read);
  Serial.print(": ");
  String msg = String(buf, BIN);
  Serial.println(msg);*/
    //byte data[bytes_to_read] = Wire.read(bytes_to_read); // receive byte as a character
    //digitalWrite(ledPin, c);
    //Serial.println(data);

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
