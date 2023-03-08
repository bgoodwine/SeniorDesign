

// Include the Wire library for I2C
#include <Wire.h>
#include "Arduino.h"

#define DEVICE_0 48
#define DEVICE_1 6
#define CURRENTPIN 4
#define CALIB 2008
#define MAXINDEX 7

union u{
  double d;
  char bytes[sizeof(double)];
};

union u current_current;
int current_request = 0;

// return average of num_readings current measurements
double readcurrent(int num_readings) {
  double current_sum = 0;
  for (int i = 0; i < num_readings; i++) {
    // Read in current sensor analog signal
    int data = analogRead(CURRENTPIN);

    // data (0, 4095) -> voltage (0, 3.3)
    double voltage = ((1.0*data)/4095.0)*3.3;

    // voltage (0.62, 1.62) -> current (-20, 20)
    current_sum = current_sum + (voltage - 1.62)*20;
  }

  // average
  double current = current_sum/num_readings;
  return current;
}

void send_current() {
  static int index = 0;
  Serial.println("Function: Send Current.");

  Serial.print("Responding with current: ");
  Serial.print(current_current.d);
  Serial.print(" A at index = ");
  Serial.print(index);
  Serial.print(" = ");
  Serial.println(current_current.bytes[index], HEX);
  Serial.println("");

  // Write current byte requested from global current measurement
  Wire.write(current_current.bytes[index]);
  index++;
  if (index > MAXINDEX) {
    index = 0;
  }
}

void requestEvent() {
  // Respond as the current request ID dictates 
  switch (current_request) {
    case 5: {
      send_current();
      break;
    }

    default: {
      Serial.print("Current request ID unknown: ");
      Serial.println(current_request);
      break;
    }
  }
}

// Request for checkin function
// Message format: "0:"
void checkin() {
  Serial.println("Function: general checkin");
}

// Restart device function (turn power on)
// Message format: "1:device_id"
void restart_device(int device_id) {
  Serial.println("Function: restart device");
  Serial.print("Device ID: ");
  Serial.println(device_id);

  switch(device_id) {
    case 0: {
      digitalWrite(DEVICE_0, LOW);
      break;
    }
    case 1: {
      digitalWrite(DEVICE_1, LOW);
      break;
    }
  }
}

// Shutdown device function (cut power)
// Message format: "2:device_id"
void shutdown_device(int device_id) {
  Serial.println("Function: shutdown device");
  Serial.print("Device ID: ");
  Serial.println(device_id);
  
  switch(device_id) {
    case 0: {
      digitalWrite(DEVICE_0, HIGH);
      break;
    }
    case 1: {
      digitalWrite(DEVICE_1, HIGH);
      break;
    }
  }
}

// Misc. message function
// Message format: "3:message_from_pi" 
// (NOTE: must be <=32B)
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
  
  // Read in data from Pi
  Serial.println("Reading...");
  Wire.readBytes(buf, num_bytes);
  
  // Convert from bytes to a string 
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
    // Print each byte recieved as it is converted
    Serial.print(hexCar);
  }

  // Convert char array to char *  
  Serial.println("");
  Serial.print("String recieved: ");
  char *msg = msg_arr;
  Serial.println(msg);
  const char *delim = ":";

  // Parse the opcode from the string
  char *op_str = strtok(msg, delim);
  int op = atoi(op_str);
  Serial.print("Opcode: ");
  Serial.println(op);

  // Parse data after the opcode
  char *data = strtok(NULL, delim);

  // Call the function associated with the opcode 
  // With appropriate parameters (sent in data)
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

    case 5: {
      Serial.println("Setting current request ID to 5");
      current_request = 5;
      break;
    }

    default:
    Serial.println("ERROR: switch default :(");
    break;

  }
}

void setup() {
  // Serial monitor initialization 
  Serial.begin(115200);
  delay(500);
  while(!Serial);

  // Join I2C bus A as slave with address 8
  uint8_t addr = 0x08; // address: 0x08
  int sda = 47; // connect SDA to Pi GPIO2
  int scl = 21; // connect SDA to Pi GPIO3

  Wire.begin(addr, sda, scl);
  Serial.println("Joined bus A.");

  // TODO: add second I2C connection here
  //int imu_sda = 10;
  //int imu_scl = 11;
  //TwoWire IMU = TwiWire(0);
  //IMU.begin(imu_sda, imu_sdl);
  
  // Register receiveEvent() as the function to run 
  // When the S3 is talked to via I2C         
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
  // Initialize "devices" (LEDs) for MOSFET control
  // And turn "on" (LOW = "on")
  pinMode(DEVICE_0, OUTPUT);
  digitalWrite(DEVICE_0, LOW);
  pinMode(DEVICE_1, OUTPUT);
  digitalWrite(DEVICE_1, LOW);

  // Initialize current sensor for analog input
  pinMode(CURRENTPIN, INPUT);
  // Initialize global current measurement
  current_current.d = 0; 

}

void loop() {
  // TODO: add IMU I2C control capabilities here (?)

  /*
  int data = analogRead(CURRENTPIN);
  Serial.print(data);

  // data (0, 4095) -> voltage (0, 3.3)
  double voltage = ((1.0*data)/4095.0)*3.3;
  Serial.print(" = ");
  Serial.print(voltage);
  Serial.print("V");

  // voltage (0.62, 1.62) -> current (-20, 20)
  double current = (voltage - 1.62)*20;
  Serial.print(" = ");
  Serial.print(current);
  Serial.println(" A");
  */

  // Read in current sensor analog signal
  current_current.d = readcurrent(10);
  /*Serial.print("Setting current current = ");
  Serial.print(current_current.d);
  Serial.println(" A");*/
  
  delay(1000);
}

