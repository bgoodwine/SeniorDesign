// Include the Wire library for I2C
#include <Wire.h>
#include "Arduino.h"
#include <Adafruit_Sensor.h>

// Private libaries
#include "PiComms.h"
#include "Tumbling.h"
#include "Sensors.h"

// External TwoWire I2C wires from Tumbling.h and PiComms.h
extern TwoWire PiBus;
extern TwoWire ESPBus;
extern Adafruit_ICM20948 icm;
extern Adafruit_MAX17048 maxlipo;

// global tumbling/anomaly values 
extern String current_anomaly_report;
extern String current_tumbling_report;
extern int current_tumbling_state;

// global current values 
extern double piIMUCurrent;
extern double piCurrent;
extern double espIMUCurrent;

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

void turnOnPi() {
  digitalWrite(Pi5_en, HIGH);
  digitalWrite(PI_IMU_en, HIGH);
  digitalWrite(LVL_SHFT_PI_EN, HIGH);
  Serial.println("Pi, Pi IMU, and Pi level shifter enabled.");
  delay(1000);
  PiBus.begin((uint8_t)ESP_ADDR, (int)PI_SDA, (int)PI_SCL);
  Serial.println("Joined Pi I2C bus.");
}

void turnOffPi() {
  // turn off pi
  // Pi low, IMU pi low, pi imu level shifter, SCL/SDA lines of pi bus 
  digitalWrite(Pi5_en, LOW);
  digitalWrite(PI_IMU_en, LOW);
  digitalWrite(LVL_SHFT_PI_EN, LOW);
  Serial.println("Pi, Pi IMU, and Pi level shifter disabled.");
}

void setup() {
  // Serial monitor initialization 
  Serial.begin(115200);
  delay(500);
  while(!Serial);
  
  // Register PiComms.h functions to handle I2C data from Pi 
  PiBus.onReceive(receiveEvent);
  PiBus.onRequest(requestEvent);

  pinMode(PR_en, OUTPUT);           // devices enable pins
  pinMode(Pi5_en, OUTPUT);
  pinMode(SDR_en, OUTPUT); 
  pinMode(IMU_en, OUTPUT);          // IMUs
  pinMode(PI_IMU_en, OUTPUT); 

  pinMode(LVL_SHFT_EN, OUTPUT);     // level shifters
  pinMode(LVL_SHFT_PI_EN, OUTPUT);
  pinMode(DCDC_EN, OUTPUT);         // DC-DC converter 

  // enable i2c communication periferals 
  digitalWrite(LVL_SHFT_EN, HIGH);
  digitalWrite(LVL_SHFT_PI_EN, HIGH);
  Serial.println("Level shifters for Pi and ESP's IMU enabled.");
  digitalWrite(DCDC_EN, HIGH);
  Serial.println("DC-DC converter enabled.");

  delay(10);

  // Begin IMU I2C connection on ESP controlled bus
  ESPBus.begin((int)ESP_SDA, (int)ESP_SCL);  
  Serial.println("Joined ESP's I2C bus.");

  // Initialize current sensor for analog input & current measurement
  pinMode(PI_IMU_CURRENT, INPUT);
  pinMode(ESP_IMU_CURRENT, INPUT);
  pinMode(PI_CURRENT, INPUT);

  // Write low all devices except IMU to initialize
  digitalWrite(PR_en, LOW);
  Serial.println("Photoresistor: OFF");
  digitalWrite(IMU_en, HIGH);
  Serial.println("IMU: ON");
  digitalWrite(SDR_en, LOW);
  Serial.println("SDR: OFF");

  // turn off pi
  turnOnPi();
  delay(1000);

  // Initialize MAX1704X (fuel gauge)
  /*if(!maxlipo.begin(&ESPBus)) {
    Serial.println("Oops, no MAX17048 detected...");
  } else {
    Serial.print(F("Found MAX17048"));
    Serial.print(F(" with Chip ID: 0x")); 
    Serial.println(maxlipo.getChipID(), HEX);
  }*/

  // Initialise IMU
  Serial.println("Adafruit ICM20948 test!");
  if (!icm.begin_I2C((uint8_t)0x69, &ESPBus, (int32_t)0)) {
    Serial.println("Failed to find ICM20948 chip");
  } else {
    Serial.println("\nSUCCESS!\n");
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
  delay(1000);
  static int i = 0;
    
  oldBatteryLevel = batteryLevel;
  
  // retreive current anomaly report
  currentState = checkState(currentState, oldCurrentState, batteryLevel);
  updateState(currentState, oldCurrentState);
  anomaly = checkAnomalies(currentState);
  batteryLevel = 0;
  String old_anomaly_report = sendReport(anomaly, oldBatteryLevel, batteryLevel);
  //Serial.print("Old anomaly report: ");
  //Serial.println(old_anomaly_report);

  oldCurrentState = currentState;

  // retreive current data as anomaly report
  current_anomaly_report = measure_imu_currents(piIMUCurrent, espIMUCurrent, piCurrent);
  current_anomaly_report = "  " + current_anomaly_report;
  Serial.println("Current anomaly report: ");
  Serial.println(current_anomaly_report);

  // save current cycle state for tumbling report
  current_tumbling_state = currentState;
  Serial.print("Current detected state: ");
  Serial.println(current_tumbling_state);
  
}
