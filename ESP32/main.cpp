// Include the Wire library for I2C
#include <Wire.h>
#include "Arduino.h"

// LED on pin 13
const int ledPin = 4;

void checkin() {
  Serial.println("Function: general checkin");
}

void restart_device(int device_id) {
  Serial.println("Function: restart device");
  Serial.print("Device ID: ");
  Serial.println(device_id);
}

void shutdown_device(int device_id) {
  Serial.println("Function: shutdown device");
  Serial.print("Device ID: ");
  Serial.println(device_id);
}

void misc_msg(int numbytes) {
  Serial.println("Function: misc. message");
  Serial.print("Num bytes: ");
  Serial.println(numbytes);
  int buflen = 32;
  byte buf[buflen];
  char msg_arr[numbytes];
  int currindex = 0;
  int msg_len = 0;

  while (numbytes > 0) {
    Wire.readBytes(buf, buflen);
    numbytes = numbytes - buflen;

    for (int i = 0; i < buflen; i++) {
      char hexCar[2];
      sprintf(hexCar, "%02X", buf[i]);
      char c = (char)buf[currindex];
      if (c != '\n') {
        msg_arr[currindex-1] = c;
        msg_len++;
        currindex++;
        Serial.print("Current index: ");
        Serial.println(currindex);
      } 
      Serial.print(hexCar); 
    }
    msg_arr[currindex] = '\0';
    Serial.println("");
    Serial.println(msg_arr);
    Serial.print("Partial string recieved: ");
    char *msg = msg_arr;
    Serial.println(msg);
    Serial.print("Numbytes left: ");
    Serial.println(numbytes);
  }

  Serial.println("FINAL MESSAGE:");
  msg_arr[currindex] = '\0';
  Serial.println("");
  Serial.print("String recieved: ");
  char *msg = msg_arr;
  Serial.println(msg);
  Serial.print("Numbytes: ");
  Serial.println(numbytes);
  
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

    case 1: {
    Serial.println("Calling 1...");
    int restart_id = atoi(data);
    restart_device(restart_id);
    break;
    }

    case 2: {
    Serial.println("Calling 2...");
    int shutdown_id = atoi(data);
    shutdown_device(shutdown_id);
    break;
    }

    case 3: {
      Serial.println("Calling 3...");
      int numbytes = atoi(data);
      misc_msg(numbytes);
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
  //Wire.begin(addr, 20, 21); // change for ESP32 S3
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
