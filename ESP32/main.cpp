

// Include the Wire library for I2C
#include <Wire.h>
#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPI.h> // needed for BNO055 library
#include <utility/imumaths.h>
#include <Math.h>

// Private libaries
#include "PiComms.h"
#include "Tumbling.h"

// External TwoWire I2C wires from Tumbling.h and PiComms.h
extern TwoWire PiBus;
extern TwoWire ESPBus;
extern Adafruit_BNO055 bno;

// Tumbling external global state variables
extern int currentState;
extern int oldCurrentState;
extern int anomaly;
extern float outputsArray[3];
extern float tumbleTime;
extern float tumbleStart;
extern float stillStart;
extern float stillTime;
extern float batteryLevel;
extern float oldBatteryLevel;
extern float battThreshold;

//extern u current_current;

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
