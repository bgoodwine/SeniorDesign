// Include the Wire library for I2C
#include <Wire.h>
#include "Arduino.h"
#include <Adafruit_Sensor.h>
//#include <Adafruit_BNO055.h>
//#include <SPI.h> // needed for BNO055 library
//#include <utility/imumaths.h>
//#include <Math.h>

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

extern u currentPi;
extern u currentESPIMU;
extern u currentPiIMU;

void setup() {
  // Serial monitor initialization 
  Serial.begin(115200);
  delay(500);
  while(!Serial);
  Serial.println("Serial begun.");

  // check all enable outputs 
  pinMode(PR_en, OUTPUT);           // devices enable pins
  pinMode(Pi5_en, OUTPUT);
  pinMode(SDR_en, OUTPUT); 
  pinMode(IMU_en, OUTPUT);          // IMUs
  pinMode(PI_IMU_en, OUTPUT); 
  pinMode(LVL_SHFT_EN, OUTPUT);     // level shifters
  pinMode(LVL_SHFT_PI_EN, OUTPUT);
  pinMode(DCDC_EN, OUTPUT);         // DC-DC converter 

  // write all enable outputs high
  digitalWrite(PR_en, HIGH);
  digitalWrite(Pi5_en, HIGH);
  digitalWrite(SDR_en, HIGH);
  digitalWrite(IMU_en, HIGH);
  digitalWrite(PI_IMU_en, HIGH);

  // enable i2c communication periferals 
  digitalWrite(LVL_SHFT_EN, HIGH);
  digitalWrite(LVL_SHFT_PI_EN, HIGH);
  Serial.println("Level shifters for Pi and ESP's IMU enabled.");
  digitalWrite(DCDC_EN, HIGH);
  Serial.println("DC-DC converter enabled.");

  // initialize current sensor pins 
  pinMode(PI_IMU_CURRENT, INPUT);
  pinMode(ESP_IMU_CURRENT, INPUT);
  pinMode(PI_CURRENT, INPUT);

  // Join I2C bus controlled by Pi as slave with address 0x08 
  PiBus.begin((uint8_t)ESP_ADDR, (int)PI_SDA, (int)PI_SCL);
  Serial.println("Joined Pi I2C bus.");
  
  // Register PiComms.h functions to handle I2C data from Pi 
  PiBus.onReceive(receiveEvent);
  PiBus.onRequest(requestEvent);

  // Begin IMU I2C connection on ESP controlled bus
  ESPBus.begin((int)ESP_SDA, (int)ESP_SCL);  

  // initialize global current values 
  currentPi.d = 0;
  currentPiIMU.d = 0;
  currentESPIMU.d = 0; 

  // Write low all devices except IMU to initialize
  /*digitalWrite(PR_en, LOW);
  Serial.println("Photoresistor: OFF");
  digitalWrite(IMU_en, HIGH);
  Serial.println("IMU: ON");
  digitalWrite(Pi5_en, LOW);
  Serial.println("Pi: OFF");
  digitalWrite(SDR_en, LOW);
  Serial.println("SDR: OFF");*/

  delay(1000);

  // Initialise IMU
  Serial.println("Adafruit ICM20948 test!");
  // Try to initialize!
  if (!icm.begin_I2C((uint8_t)0x68, &ESPBus, (int32_t)0)) {
    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }

    // Initialize MAX1704X (fuel gauge)
  if(!maxlipo.begin(&ESPBus)) {
    Serial.println("Oops, no MAX17048 detected...");
    while (1);
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
  /*delay(3000);
  
  // TODO: Read in current sensor analog signal for BOTH
  //currentPi.d = readcurrent(10);

  double pi_imu_current;
  double esp_imu_current;
  double pi_current;
  measure_imu_currents(pi_imu_current, esp_imu_current, pi_current);
  Serial.println("Currents measured.");

  // set global current values 
  currentPiIMU.d = pi_imu_current; 
  currentESPIMU.d = esp_imu_current;
  currentPi.d = pi_current;
  
  oldBatteryLevel = batteryLevel;*/
  
  // Check anomaly/state and send report
  /*currentState = checkState(currentState, oldCurrentState, batteryLevel);
  updateState(currentState, oldCurrentState);
  anomaly = checkAnomalies(currentState);*/
  
  /*batteryLevel = maxlipo.cellPercent();
  Serial.print("Battery percentage:  ");
  Serial.print(batteryLevel);
  Serial.println(" %");*/
  
  /*current_report = sendReport(anomaly, oldBatteryLevel, batteryLevel);
  Serial.print("Message for I2C transfer to Pi: ");
  Serial.println(current_report);
  current_report = " " + current_report;

  oldCurrentState = currentState;*/
}
