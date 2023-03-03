

// Include the Wire library for I2C
#include <Wire.h>
#include "Arduino.h"

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

void misc_msg(char *data) {
  Serial.println("Function: misc. message");
  Serial.print("Message: ");
  Serial.println(data);
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
      misc_msg(data);
      break;
    }

    default:
    Serial.println("ERROR: switch default :(");
    break;

  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  while(!Serial);

  // Join I2C bus as slave with address 8
  uint8_t addr = 0x08; // address: 0x08
  int sda = 47; // connect to Pi GPIO2
  int scl = 21; // connect to Pi GPIO3
  
  //Wire.setSpeed(10);
  Wire.begin(addr, sda, scl);

  // add to platformio.ini:
  // monitor_speed = 115200
  Serial.println("Joined bus...");
  
  // Call receiveEvent when data received                
  Wire.onReceive(receiveEvent);
  
  // Setup pin 13 as output and turn LED off
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}
void loop() {
}

