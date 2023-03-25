

// Include the Wire library for I2C
#include <Wire.h>
#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPI.h> // needed for BNO055 library
#include <utility/imumaths.h>
#include <Math.h>

// state detection definitions 
#define undeployed 0
#define tumblingState 1
#define dayCycle 2
#define nightCycle 3
#define lowPower 4
#define downlink 5
#define error 6
#define tumbling 1
#define notTumbling 0

// i2c definitions 
#define PI_SDA 47 // Pi Zero SDA = GPIO2
#define PI_SCL 21 // Pi Zero SCL = GPIO3
#define ESP_ADDR 0x08 // ESP32 address
#define MAXINDEX 7
#define ESP_SDA 38 // ESP controlled I2C bus
#define ESP_SCL 37 // ESP controlled I2C bus

// device on/off pin definitions 
#define DEVICE_0 48
#define DEVICE_1 6
#define CURRENTPIN 4

//#define batt_in A0
//#define currSense_in A1
#define PR_in 15
#define IMU_in 16
#define Pi5_in 17
#define SDR_in 18
#define dl_in 8

#define PR_en 9
#define IMU_en 10
#define Pi5_en 11
#define SDR_en 12

#define battAnomaly 1
#define IMUAnomaly 2
#define SDRAnomaly 3
#define Pi5Anomaly 4
#define noAnomaly 0

// Initialize I2C busses & devices 
TwoWire PiBus = TwoWire(0); // I2C bus for Pi to control
TwoWire ESPBus = TwoWire(1); // I2C bus for ESP32 to control
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &ESPBus);

// Initialize state, anomaly, outputs array
int currentState;
int oldCurrentState = undeployed;
int anomaly = noAnomaly;
float outputsArray[3] = {0,0,0};

// Initialize tumbling detection variables
float tumbleTime = 0;
float tumbleStart = 0;
float stillStart = 0;
float stillTime = 0;

// Initialize battery level
float batteryLevel = 100.0;
float oldBatteryLevel = 0.0;
float battThreshold = 10.0;

union u{
  double d;
  char bytes[sizeof(double)];
};

union u current_current;
int current_request = 0;

void sendReport(int anomaly, float oldBatteryLevel, float batteryLevel) {
  String message;
  // need to add support for multiple anomalies

  int battLev = int(floor(batteryLevel));
//  Serial.print("batteryLevel: ");
//  Serial.println(batteryLevel);
//  Serial.print("battLev: ");
//  Serial.println(battLev);
  int oldBattLev = int(floor(oldBatteryLevel));
  
  if (anomaly == battAnomaly){
    message = "Anomaly detected: battery";
    Serial.println(message);
  }
  else if (anomaly == IMUAnomaly){
    message = "Anomaly detected: IMU";
    Serial.println(message);
  }
  else if (anomaly == Pi5Anomaly){
    message = "Anomaly detected: Pi5V";
    Serial.println(message);
  }
  else if (anomaly == SDRAnomaly){
    message = "Anomaly detected: SDR";
    Serial.println(message);
  }

  if (battLev != oldBattLev){
    message = "Battery Level: ";
    Serial.print(message);
    Serial.println(battLev);
  }
//  else if (anomaly == noAnomaly){
//    message = "No anomaly detected";
//  }

//  Serial.print("Message for I2C transfer to Pi: ");
  
}

void tumblingDetection(float (& outputsArray)[3])
{
  //Serial.println("Tumbling detection");
  // fix this function. Need to pass in tumble time as an input
  
  int tumbleState = notTumbling;
  int tumbleStart = outputsArray[2];
  int stillStart = outputsArray[3];
  /* Beginning of Tumbling Detection Function*/
  sensors_event_t angVelocityData;
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

  double x = -1000000, y = -1000000 , z = -1000000;

  x = angVelocityData.acceleration.x;
  y = angVelocityData.acceleration.y;
  z = angVelocityData.acceleration.z;

    /* Display the floating point data */  
//  Serial.print("X: ");
//  Serial.print(x);
//  Serial.print("\tY: ");
//  Serial.print(y);
//  Serial.print("\tZ: ");
//  Serial.print(z);
//  Serial.println("");
//  Serial.print("TumbleTime: ");
//  Serial.println(tumbleTime);
//  Serial.print("TumbleStart: ");
//  Serial.println(tumbleStart);

  if ((abs(x) > 1) || (abs(y) > 1) || (abs(z) > 1)) {
    
    stillStart = 0;
    
    if ((tumbleState == notTumbling)&&(tumbleStart == 0)){
      tumbleStart = millis();
    }
    else {
      tumbleTime = millis()-tumbleStart;
      if (tumbleTime > 3000){
        tumbleState = tumbling;
        tumbleTime = 0;
        tumbleStart = 0;
      }
    }
  }
  else {
    if (tumbleState == tumbling) {
      tumbleStart = 0;
      
      if (stillStart == 0){
        stillStart = millis();
      }
      else {
        stillTime = millis()-stillStart;
        if (stillTime > 3000){
          tumbleState = notTumbling;
          stillTime = 0;
          stillStart = 0;
        }
      }
    }
  }

  // problem with tumble state being = 128
//  Serial.print("State from tumbling detection: ");
//  Serial.println(tumbleState);
  
  tumbleState = float(tumbleState);
  outputsArray[1] = tumbleState;
  outputsArray[2] = tumbleStart;
  outputsArray[3] = stillStart;
//  Serial.println(outputsArray[1]);

  /* End of Tumbling Detection Function*/
}

void updateState(int currentState, int oldCurrentState) {

  if (oldCurrentState != currentState){
    if (currentState == tumblingState){
      // turn on motor board
      digitalWrite(Pi5_en, HIGH);
      Serial.println("Pi: ON");
  //    digitalWrite(Pi33_en, HIGH);
      digitalWrite(SDR_en, LOW);
      Serial.println("SDR: OFF");
      digitalWrite(PR_en, HIGH);
      Serial.println("Photoresistor: ON");
    }
    else if (currentState == dayCycle){
      // turn on motor board
      digitalWrite(PR_en, HIGH);
      Serial.println("Photoresistor: ON");
      digitalWrite(Pi5_en, HIGH);
      Serial.println("Pi: ON");
  //    digitalWrite(Pi33_en, HIGH);
      digitalWrite(SDR_en, LOW);
      Serial.println("SDR: OFF");
      digitalWrite(IMU_en, HIGH);
      Serial.println("IMU: ON");
    }
    else if (currentState == nightCycle){
      // turn off motor board
      digitalWrite(PR_en, HIGH);
      Serial.println("Photoresistor: ON");
  //    digitalWrite(Pi33_en, LOW);
      digitalWrite(Pi5_en, LOW);
      Serial.println("Pi: OFF");
      digitalWrite(SDR_en, LOW);
      Serial.println("SDR: OFF");
      digitalWrite(IMU_en, LOW);
      Serial.println("IMU: OFF");
    }
    else if (currentState == lowPower){
      // turn off motor board
      digitalWrite(PR_en, LOW);
      Serial.println("Photoresistor: OFF");
  //    digitalWrite(Pi33_en, LOW);
      digitalWrite(Pi5_en, LOW);
      Serial.println("Pi: OFF");
      digitalWrite(SDR_en, LOW);
      Serial.println("SDR: OFF");
      digitalWrite(IMU_en, LOW);
      Serial.println("IMU: OFF");
    }
    else if (currentState == downlink){
      // turn on motor board
      digitalWrite(PR_en, HIGH);
      Serial.println("Photoresistor: ON");
  //    digitalWrite(Pi33_en, HIGH);
      digitalWrite(Pi5_en, HIGH);
      Serial.println("Pi: ON");
      digitalWrite(SDR_en, HIGH);
      Serial.println("SDR: ON");
      digitalWrite(IMU_en, HIGH);
      Serial.println("IMU: ON"); 
    }
    else if (currentState == lowPower){
      // turn on motor board
      digitalWrite(PR_en, LOW);
      Serial.println("Photoresistor: OFF");
  //    digitalWrite(Pi33_en, HIGH);
      digitalWrite(Pi5_en, LOW);
      Serial.println("Pi: OFF");
      digitalWrite(SDR_en, LOW);
      Serial.println("SDR: OFF");
      digitalWrite(IMU_en, LOW);
      Serial.println("IMU: OFF"); 
    }
    else if (currentState == error){
      Serial.println("State update error");
    }
  }
  
}

int checkAnomalies(int currentState) {
  /* Read in all voltages, check if they are
     in operating range from datasheeet */

  bool IMUon;
  bool Pion;
  bool SDRon;

  if ((currentState == dayCycle) || (currentState == downlink) || (currentState == tumblingState)){
    Pion = true;
  }
  else {
    Pion = false;
  }

  if ((currentState != lowPower)&&(currentState != nightCycle)){
    IMUon = true;
  }
  else {
    IMUon = false;
  }

  if ((currentState == downlink)){
    SDRon = true;
  }
  else {
    SDRon = false;
  }

  int battv;
  int IMUv;
  int Pi5v;
  int SDRv;

  //battv = analogRead(batt_in);
  IMUv = analogRead(IMU_in);
  Pi5v = digitalRead(Pi5_in);
  SDRv = digitalRead(SDR_in);

//  Serial.print("SDRv: ");
//  Serial.println(SDRv);

//  Serial.println("");
//  Serial.println("Battery voltage");
//  Serial.println(battv);
//  Serial.println("IMU voltage");
//  Serial.println(IMUv);
//  Serial.println("Pi5 voltage");
//  Serial.println(Pi5v);
//  Serial.println("");

  //if (battv < floor(200.0*3)) {
  //  return battAnomaly;
  //}
  if (((IMUv < floor(200.0*2.4))||(IMUv > floor(200.0*3.6)))&&(IMUon)) {
    return IMUAnomaly;
  }
//  else if (((Pi5v < floor(4.75*200.0))||(Pi5v > floor(5.25*200.0)))&&(Pion)) {
  else if ((Pi5v < 1)&&(Pion)) {
    return Pi5Anomaly;
  }
//  else if (((SDRv < floor(4.75*200.0))||(SDRv > floor(5.25*200.0)))&&(SDRon)) {
  else if ((SDRv < 1)&&(SDRon)) {
    return SDRAnomaly;
  }
  else {
    return noAnomaly;
  }
  
}

int dayCycleCheck() {
  int val;
  val = analogRead(PR_in);
//  Serial.print("PR voltage: ");
//  Serial.println(val);

  if (val > 750){
    return 1;
  }
  else {
    return 0;
  }
}

int checkState(int currentState, int oldCurrentState) {
  int dayCycleBool;
  int lowPowerBool;
  int downlinkBool;
  int tumblingBool = 0;

  if ((currentState == undeployed)||(currentState == tumbling)){
    tumblingDetection(outputsArray); //might have to modify this to run in the background
    tumblingBool = outputsArray[1];
    tumblingBool = int(tumblingBool);
  }

  if (currentState == undeployed) {
    if (!tumblingBool) {
      currentState = undeployed;
//      if (currentState != oldCurrentState){
//        Serial.println("Undeployed state detected");
//      }
    }
    else {
      currentState = tumblingState;
      if (currentState != oldCurrentState){
        Serial.println("Tumbling state detected");
      }
    } 
  }
  else if (!tumblingBool){
//    Serial.println("Boolcheck");
    dayCycleBool = dayCycleCheck();
    //lowPowerBool = lowPowerCheck();
    //downlinkBool = downlinkCheck();
    lowPowerBool = false;    
    downlinkBool = false;

    if ((dayCycleBool)&&(!downlinkBool)&&(!lowPowerBool)){
      currentState = dayCycle;
      if (currentState != oldCurrentState){
        Serial.println("Day cycle detected");
      }
    }
    else if ((downlinkBool)&&(!lowPowerBool)){
      currentState = downlink;
      if (currentState != oldCurrentState){
        Serial.println("Downlink state detected");
      }
    }
    else if (lowPowerBool){
      currentState = lowPower;
      if (currentState != oldCurrentState){
        Serial.println("Low power detected");
      }
    }
    else if ((!dayCycleBool)&&(!downlinkBool)&&(!lowPowerBool)){
      currentState = nightCycle;
      if (currentState != oldCurrentState){
        Serial.println("Night cycle detected");
      }
    }
    else {
      currentState = error;
      if (currentState != oldCurrentState){
        Serial.println("State error detected");
      }
    }
  }
  else {
    currentState = tumblingState;
    if (currentState != oldCurrentState){
      Serial.println("Tumbling State detected");
    }
  }
  
//  Serial.print("Current State: ");
//  Serial.println(currentState);

  return currentState;
}

int8_t temperature_read() {
  int8_t boardTemp = bno.getTemp();
  return boardTemp;
}

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
      //checkin();
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

void setup() {
  // Serial monitor initialization 
  Serial.begin(115200);
  delay(500);
  while(!Serial);

  // Join I2C bus A as slave with address 8
  // NOTE: casting resolves overloading ambiguity!
  //Wire.begin((uint8_t)ESP_ADDR, (int)PI_SDA, (int)PI_SCL);
  //Serial.println("Joined bus A.");

  // TODO: add second I2C connection here
  
  PiBus.begin((uint8_t)ESP_ADDR, (int)PI_SDA, (int)PI_SCL);
  Serial.println("Joined Pi I2C bus.");
  
  // Register receiveEvent() as the function to run 
  // When the S3 is talked to via I2C         
  PiBus.onReceive(receiveEvent);
  PiBus.onRequest(requestEvent);

  // Begin IMU I2C connection on ESP controlled bus
  ESPBus.begin((int)ESP_SDA, (int)ESP_SCL);  
  if(!bno.begin())
  {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  else {
    bno.setExtCrystalUse(true);
    Serial.println("Joined ESP32 I2C bus.");
  }

  // Initialize "devices" (LEDs) for MOSFET control & turn "on" (LOW)
  pinMode(DEVICE_0, OUTPUT);
  digitalWrite(DEVICE_0, LOW);
  pinMode(DEVICE_1, OUTPUT);
  digitalWrite(DEVICE_1, LOW);

  // Initialize current sensor for analog input
  pinMode(CURRENTPIN, INPUT);
  // Initialize global current measurement
  current_current.d = 0; 

  // Initialize tumbling detection output pins
  pinMode(PR_en, OUTPUT);
  pinMode(IMU_en, OUTPUT);
  pinMode(Pi5_en, OUTPUT);
  pinMode(SDR_en, OUTPUT);

  // Initialize tumbling detection input pins
  pinMode(dl_in, INPUT);
  pinMode(Pi5_in, INPUT);
  pinMode(SDR_in, INPUT);

  // Write low all devices except IMU to initialize
  digitalWrite(PR_en, LOW);
  Serial.println("Photoresistor: OFF");
  digitalWrite(IMU_en, HIGH);
  Serial.println("IMU: ON");
  digitalWrite(Pi5_en, LOW);
  Serial.println("Pi: OFF");
  digitalWrite(SDR_en, LOW);
  Serial.println("SDR: OFF");

  Serial.println("Anomaly/State Detection System Initialized");
  Serial.println("Undeployed state detected");

}

void loop() {
  // Read in current sensor analog signal
  /*current_current.d = readcurrent(10);
  delay(1000);*/
  
  // TODO: add IMU I2C control capabilities here (?)
  delay(3000);

  //oldBatteryLevel = batteryLevel;
  
  currentState = checkState(currentState, oldCurrentState);
  updateState(currentState, oldCurrentState);
  anomaly = checkAnomalies(currentState);
  //batteryLevel = checkBatteryLevel();
  batteryLevel = -1;
  sendReport(anomaly, oldBatteryLevel, batteryLevel);

  oldCurrentState = currentState;
}
