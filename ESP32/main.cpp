// Include the Wire library for I2C
#include <Wire.h>
#include "Arduino.h"

// LED on pin 13
const int ledPin = 4;

void checkin() {
  Serial.println("Function: general checkin");
}

void imu_checkin(char *data) {
  Serial.println("Function: IMU checkin");
  Serial.print("Data: ");
  Serial.println(data);
}
void transmit_sdr_message(char *message) {
  Serial.println("Function: send message to SDR");
  Serial.print("Message: ");
  Serial.println(message);
}

void restart_device(int device_id) {
  Serial.println("Function: restart device");
  Serial.print("Device ID: ");
  Serial.println(device_id);
}

// Function that executes whenever data is received from master
void receiveEvent(int bytes_to_read) {
  int num_bytes = 32;
  byte buf[num_bytes];
  char msg_arr[num_bytes];
  int msg_len = 0;
  
  Serial.println("Reading...");
  Wire.readBytes(buf, num_bytes);
  
  Serial.print("Bytes received: ");
  for (int i = 0; i < num_bytes; i++) {
    char hexCar[2];
    sprintf(hexCar, "%02X", buf[i]);
    char c = (char)buf[i];
    if (c != '\n' && i != 0) {
      msg_arr[i-1] = c;
      msg_len++;
    } else {
      msg_arr[i] = '\0';
    }
    Serial.print(hexCar);
  }

  Serial.println("");
  Serial.print("String recieved: ");
  char *msg = msg_arr;
  Serial.println(msg);
  const char *delim = ":";

  char *op_str = strtok(msg, delim);
  int op = atoi(op_str);
  Serial.print("Opcode: ");
  Serial.println(op);

  char *data = strtok(NULL, delim);

  switch(op) {
    case 0: 
    Serial.println("Calling 0...");
    checkin();
    break;

    case 1:
    Serial.println("Calling 1...");
    imu_checkin(data);
    break;

    case 2:
    Serial.println("Calling 2...");
    transmit_sdr_message(data);
    break;

    case 3:
    {
      Serial.println("Calling 3...");
      int device_id = atoi(data);
      restart_device(device_id);
      break;
    }

    default:
    Serial.println("ERROR: switch default :(");
    break;

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
