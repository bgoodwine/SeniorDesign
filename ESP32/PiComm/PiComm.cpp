#include "PiComms.h"
  
int current_request = 0;
u currentPiIMU = {0};
u currentESPIMU = {0};
u currentPi = {0};
String current_report = "";
TwoWire PiBus = TwoWire(0);
extern float batteryLevel;

// return average of num_readings current measurements
/*double readcurrent(int num_readings) {
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
}*/

void send_sdr_msg() {
  static int index = 20;
  String msg = "Fake message for SDR :)";

  Serial.print("Responding with char at index = ");
  Serial.print(index);
  Serial.print(" = ");
  Serial.println(msg[index]);

  PiBus.write(msg[index]);
  index++;
  if (index > msg.length()) {
    index = 0;
  }
}

// Request for checkin function
// Message format: "0:"
void checkin() {
  static String report = "";
  static char reportarray[BUFSIZ];

  static int index = 0;
  if (index == 0) {
    current_report.toCharArray(reportarray, current_report.length()+1);
    report = String(reportarray);
  }

  Serial.println("Function: general checkin");
  Serial.print("Message for Pi: ");
  Serial.println(report);

  Serial.print("Responding with message: ");
  Serial.print(report);
  Serial.print("Of length: ");
  Serial.println(report.length());
  Serial.print(" A at index = ");
  Serial.print(index);
  Serial.print(" = ");
  Serial.print(report[index]);
  Serial.print(" = ");
  Serial.println(report[index], HEX);
  Serial.println("");

  // Write current byte requested from global current measurement
  PiBus.write(report[index]);
  index++;
  if (index > report.length()) {
    index = 0;
    Serial.println("RESETTING STATUS INDEX");
  }
}

void send_current() {
  static int index   = 0;
  int maxindex = sizeof(double);
  int indexPi = index; 
  int indexIMU = index - maxindex; 
  int indexBatt = index - maxindex*2;
  static u reportCurrentPi;
  static u reportCurrentIMU;
  static u batteryLevel;

  if (indexPi == 0 && indexIMU == 0 && indexBatt == 0) {
    reportCurrentPi.d = currentPiIMU.d;
    reportCurrentIMU.d = currentESPIMU.d;
    batteryLevel.d = (double)maxlipo.cellPercent();
  }

  // 0-7 -> send Pi current
  if (indexPi < maxindex) {
    Serial.print("Pi current: ");
    Serial.print(reportCurrentPi.d);
    Serial.print(" at pi index = ");
    Serial.print(indexPi);
    Serial.print(" & index = ");
    Serial.println(index);
    /*Serial.print(" = ");
    Serial.println(currentPi.bytes[indexPi], HEX);
    Serial.println("");*/
    PiBus.write(reportCurrentPi.bytes[indexPi]);
  // 8-15 = 0-7 -> send IMU current 
  } else if (indexIMU < maxindex) {
    Serial.print("Responding with IMU current: ");
    Serial.print(reportCurrentIMU.d);
    Serial.print(" at IMU index = ");
    Serial.print(indexIMU);
    Serial.print(" & index = ");
    Serial.println(index);
    /*Serial.print(" = ");
    Serial.println(currentIMU.bytes[indexIMU], HEX);
    Serial.println("");*/
    PiBus.write(reportCurrentIMU.bytes[indexIMU]);
  // 16-23 = 0-7 -> send battery %
  } else if (indexBatt < maxindex) {
    Serial.print("Responding with Battery Percentage: ");
    Serial.print(batteryLevel.d);
    Serial.print(" at batt index = ");
    Serial.print(indexBatt);
    Serial.print(" & index = ");
    Serial.println(index);
    /*Serial.print(" = ");
    Serial.println(batteryLevel.bytes[indexBatt], HEX);
    Serial.println("");*/
    // Write current byte requested from global current measurement
    PiBus.write(batteryLevel.bytes[indexBatt]);
  }
  
  index++;

  if (index >= maxindex*3) {
    index = 0;
    Serial.println("INDEX RESET");
  }

  /*static int index   = 0;
  int indexPi = index;
  int indexIMU = index - MAXINDEX;
  int indexBatt = index - MAXINDEX*2;
  static u reportCurrentPi;
  static u reportCurrentIMU;
  static u batteryLevel;

  if (indexPi == 0 && indexIMU == 0 && indexBatt == 0) {
    reportCurrentPi.d = currentPi.d;
    reportCurrentIMU.d = currentIMU.d;
    batteryLevel.d = (double)maxlipo.cellPercent();
  }
  Serial.println("Function: Send Current.");

  if (indexPi <= MAXINDEX) {
    Serial.print("Responding with PI current: ");
    Serial.print(currentPi.d);
    Serial.print(" A at index = ");
    Serial.print(indexPi);
    Serial.print(" = ");
    Serial.println(currentPi.bytes[indexPi], HEX);
    Serial.println("");
    // Write current byte requested from global current measurement
    PiBus.write(currentPi.bytes[indexPi]);
  } else if (indexIMU <= MAXINDEX*2) {
    Serial.print("Responding with IMU current: ");
    Serial.print(currentIMU.d);
    Serial.print(" A at index = ");
    Serial.print(indexIMU);
    Serial.print(" = ");
    Serial.println(currentIMU.bytes[indexIMU], HEX);
    Serial.println("");
    // Write current byte requested from global current measurement
    PiBus.write(currentIMU.bytes[indexIMU]);
  } else if (indexBatt <= MAXINDEX*3) {
    Serial.print("Responding with Battery Percentage: ");
    Serial.print(batteryLevel.d);
    Serial.print(" A at index = ");
    Serial.print(indexBatt);
    Serial.print(" = ");
    Serial.println(batteryLevel.bytes[indexBatt], HEX);
    Serial.println("");
    // Write current byte requested from global current measurement
    PiBus.write(batteryLevel.bytes[indexBatt]);
  }
  
  if (index >= MAXINDEX*3) {
    //indexPi = 0;
    index = 0;
    Serial.println("Pi is DONE");
  } else {
    index++;
  }*/

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
      //digitalWrite(DEVICE_0, LOW);
      Serial.println("TURN ON DEVICE 0 REMOVED");
      break;
    }
    case 1: {
      //digitalWrite(DEVICE_1, LOW);
      Serial.println("TURN ON DEVICE 1 REMOVED");
      break;
    }
    case 2: {
      //digitalWrite(PR_en, HIGH);
      Serial.println("Wish I could be Turning photoresistor H");

      break;
    }
    case 3: {
      //digitalWrite(IMU_en, HIGH);
      Serial.println("Wish I could be Turning IMU H");
      break;
    }
    case 4: {
      //digitalWrite(Pi5_en, HIGH);
      Serial.println("Wish I could be Turning Pi 5 H");
      break;
    }
    case 5: {
      //digitalWrite(SDR_en, HIGH);
      Serial.println("Wish I could be Turning SDR H");
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
      //digitalWrite(DEVICE_0, HIGH);
      Serial.println("DEVICE 0 GONE");
      break;
    }
    case 1: {
      //digitalWrite(DEVICE_1, HIGH);
      Serial.println("DEVICE 1 GONE");
      break;
    }
    case 2: {
      //digitalWrite(PR_en, LOW);
      Serial.println("Wish I could be Turning photoresistor L");
      break;
    }
    case 3: {
      //digitalWrite(IMU_en, LOW);
      Serial.println("Wish I could be Turning IMU L");
      break;
    }
    case 4: {
      //digitalWrite(Pi5_en, LOW);
      Serial.println("Wish I could be Turning Pi 5 L");
      break;
    }
    case 5: {
      //digitalWrite(SDR_en, LOW);
      Serial.println("Wish I could be Turning SDR L");
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
