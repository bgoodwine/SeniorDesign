#include "PiComms.h"
  
int current_request = 0;
u current_current = {0};
String current_report = "";
TwoWire PiBus = TwoWire(0);

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

void send_sdr_msg() {
  static int index = 0;
  String msg = "Fake message for SDR :)";

  Serial.print("Responding with char at index = ");
  Serial.print(index);
  Serial.print(" = ");
  Serial.println(msg[index]);

  PiBus.write(msg[index]);
  index++;
  if (index >= msg.length()) {
    index = 0;
  }
}

// Request for checkin function
// Message format: "0:"
void checkin() {
  static int index = 0;
  Serial.println("Function: general checkin");
  Serial.print("Message for Pi: ");
  Serial.println(current_report);

  Serial.print("Responding with message: ");
  Serial.print(current_report);
  Serial.print(" A at index = ");
  Serial.print(index);
  Serial.print(" = ");
  Serial.print(current_report[index]);
  Serial.print(" = ");
  Serial.println(current_report[index], HEX);
  Serial.println("");

  // Write current byte requested from global current measurement
  PiBus.write(current_report[index]);
  index++;
  if (index > current_report.length() + 1) {
    index = 0;
  }
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
  PiBus.write(current_current.bytes[index]);
  index++;
  if (index > MAXINDEX) {
    index = 0;
  }
}

void requestEvent() {
  // Respond as the current request ID dictates 
  switch (current_request) {
    case 0: {
      // 0 - request for general checkin
      checkin();
      break;
    }
    case 1: {
      // 1 - send current reading from ESP32 to Pi
      send_current();
      break;
    }
    case 2: {
      // 2 - send message from ESP32 for SDR to Pi
      send_sdr_msg();
      break;

    }

    default: {
      Serial.print("Current request ID unknown: ");
      Serial.println(current_request);
      break;
    }
  }
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
    case 2: {
      digitalWrite(PR_en, HIGH);
      Serial.println("Turning photoresistor H");

      break;
    }
    case 3: {
      digitalWrite(IMU_en, HIGH);
      Serial.println("Turning IMU H");
      break;
    }
    case 4: {
      digitalWrite(Pi5_en, HIGH);
      Serial.println("Turning Pi 5 H");
      break;
    }
    case 5: {
      digitalWrite(SDR_en, HIGH);
      Serial.println("Turning SDR H");
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
    case 2: {
      digitalWrite(PR_en, LOW);
      Serial.println("Turning photoresistor L");
      break;
    }
    case 3: {
      digitalWrite(IMU_en, LOW);
      Serial.println("Turning IMU L");
      break;
    }
    case 4: {
      digitalWrite(Pi5_en, LOW);
      Serial.println("Turning Pi 5 L");
      break;
    }
    case 5: {
      digitalWrite(SDR_en, LOW);
      Serial.println("Turning SDR L");
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
  PiBus.readBytes(buf, num_bytes);
  
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

    case 4: {
      current_request = atoi(data);
      Serial.print("Setting current request ID to ");
      Serial.println(current_request);
      break;
    }

    default:
    Serial.println("ERROR: switch default :(");
    break;

  }
}
