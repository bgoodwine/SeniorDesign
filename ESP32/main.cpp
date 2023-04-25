// Include the Wire library for I2C
#include <Wire.h>
#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
/*#include <SPI.h> // needed for BNO055 library
#include <utility/imumaths.h>
#include <Math.h>*/

// Private libaries
#include "PiComms.h"
#include "Tumbling.h"
#include "Sensors.h"

// External TwoWire I2C wires from Tumbling.h and PiComms.h
extern TwoWire PiBus;
extern TwoWire ESPBus;
extern Adafruit_ICM20948 icm;
extern Adafruit_MAX17048 maxlipo;

extern String current_report;

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

void setup() {
  // Serial monitor initialization 
  Serial.begin(115200);
  delay(500);
  while(!Serial);

  // Join I2C bus controlled by Pi as slave with address 0x08 
  // NOTE: casting resolves overloading ambiguity!
  PiBus.begin((uint8_t)ESP_ADDR, (int)PI_SDA, (int)PI_SCL);
  Serial.println("Joined Pi I2C bus.");
  
  // Register PiComms.h functions to handle I2C data from Pi 
  PiBus.onReceive(receiveEvent);
  PiBus.onRequest(requestEvent);

  // Begin IMU I2C connection on ESP controlled bus
  ESPBus.begin((int)ESP_SDA, (int)ESP_SCL);  

  // Initialize "devices" (LEDs) for MOSFET control & turn "on" (LOW)
  pinMode(DEVICE_0, OUTPUT);
  pinMode(DEVICE_1, OUTPUT);
  digitalWrite(DEVICE_0, LOW);
  digitalWrite(DEVICE_1, LOW);

  // Initialize current sensor for analog input & current measurement
  pinMode(CURRENTPIN, INPUT);
  currentPi.d = 0;
  currentIMU.d = 0; 

  // Initialize tumbling detection output pins
  pinMode(PR_en, OUTPUT);
  pinMode(IMU_en, OUTPUT);
  pinMode(Pi5_en, OUTPUT);
  pinMode(SDR_en, OUTPUT);

  // Write low all devices except IMU to initialize
  digitalWrite(PR_en, LOW);
  Serial.println("Photoresistor: OFF");
  digitalWrite(IMU_en, HIGH);
  Serial.println("IMU: ON");
  digitalWrite(Pi5_en, LOW);
  Serial.println("Pi: OFF");
  digitalWrite(SDR_en, LOW);
  Serial.println("SDR: OFF");

  delay(1000);

  /* Initialise IMU */
  Serial.println("Adafruit ICM20948 test!");
  // Try to initialize!
  if (!icm.begin_I2C((uint8_t)0x68, &ESPBus, (int32_t)0)) {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }

    // Initialize MAX1704X (fuel gauge)
  if(!maxlipo.begin(&ESPBus)) {
    Serial.println("Oops, no MAX17048 detected...");
  } else {
    Serial.print(F("Found MAX17048"));
    Serial.print(F(" with Chip ID: 0x")); 
    Serial.println(maxlipo.getChipID(), HEX);
  }

  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange()) {
  case ICM20948_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  }

  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);

  Serial.println("I2C connections ready.");
  Serial.println("Device Control System Initialized.");
  Serial.println("Anomaly/State Detection System Initialized.");
  Serial.println("Undeployed state detected...");

}

void loop() {
  delay(3000);
  
  // TODO: Read in current sensor analog signal for BOTH
  currentPi.d = readcurrent(10);
  currentIMU.d = currentPi.d; 
  
  oldBatteryLevel = batteryLevel;
  
  // Check anomaly/state and send report
  currentState = checkState(currentState, oldCurrentState, batteryLevel);
  updateState(currentState, oldCurrentState);
  anomaly = checkAnomalies(currentState);
  batteryLevel = maxlipo.cellPercent();
  // Serial.print(batteryLevel);
  
  current_report = sendReport(anomaly, oldBatteryLevel, batteryLevel);
  Serial.print("Message for I2C transfer to Pi: ");
  Serial.println(current_report);
  current_report = " " + current_report;

  oldCurrentState = currentState;
}
