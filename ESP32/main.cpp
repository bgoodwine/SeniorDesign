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

// External TwoWire I2C wires from Tumbling.h and PiComms.h
extern TwoWire PiBus;
extern TwoWire ESPBus;
extern Adafruit_BNO055 bno;
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

  // Initialize MAX1704X (fuel gauge)
  if(!maxlipo.begin(&ESPBus)) {
    Serial.println("Oops, no MAX17048 detected...");
  } else {
    Serial.print(F("Found MAX17048"));
    Serial.print(F(" with Chip ID: 0x")); 
    Serial.println(maxlipo.getChipID(), HEX);
  }

  // Initialize BNO055 (IMU)
  
  if(!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  else {
    bno.setExtCrystalUse(true);
    Serial.println("Found BNO55.");
  }

  // Initialize "devices" (LEDs) for MOSFET control & turn "on" (LOW)
  pinMode(DEVICE_0, OUTPUT);
  pinMode(DEVICE_1, OUTPUT);
  digitalWrite(DEVICE_0, LOW);
  digitalWrite(DEVICE_1, LOW);

  // Initialize current sensor for analog input & current measurement
  pinMode(CURRENTPIN, INPUT);
  current_current.d = 0; 

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
  
  // Initialize tumbling detection input pins
  pinMode(dl_in, INPUT);
  pinMode(Pi5_in, INPUT);
  pinMode(SDR_in, INPUT);

  Serial.println("Device Control System Initialized.");
  Serial.println("Anomaly/State Detection System Initialized.");
  Serial.println("Undeployed state detected...");

}

void loop() {
  delay(3000);
  
  // Read in current sensor analog signal
  current_current.d = readcurrent(10);
  
  oldBatteryLevel = batteryLevel;
  
  // Check anomaly/state and send report
  currentState = checkState(currentState, oldCurrentState);
  updateState(currentState, oldCurrentState);
  anomaly = checkAnomalies(currentState);
  batteryLevel = maxlipo.cellPercent();
  
  current_report = sendReport(anomaly, oldBatteryLevel, batteryLevel);
  Serial.print("Message for I2C transfer to Pi: ");
  Serial.println(current_report);

  oldCurrentState = currentState;
}
