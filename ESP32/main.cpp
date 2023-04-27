// Include the Wire library for I2C
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
/*#include <SPI.h> // needed for BNO055 library
#include <utility/imumaths.h>
#include <Math.h>*/

// Private libaries
#include "PiComms.h"
#include "Tumbling.h"
#include "Sensors.h"



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(2000);
  //while(!Serial);
  Serial.println("Serial begun.");
  //ESPBus.begin((int)ESP_SDA, (int)ESP_SCL);
  ESPBus.begin((int)PI_SDA, (int)PI_SCL);    
  Serial.println("Joined ESP32 (PI) bus.");

  pinMode(DCDC_EN, INPUT); // 1.8 DC-DC converter will be ~3.3V 
  pinMode(PI_IMU_en, OUTPUT);
  digitalWrite(PI_IMU_en, HIGH);
  Serial.println("Pi's IMU enabled.");
  pinMode(LVL_SHFT_PI_EN, OUTPUT);
  digitalWrite(LVL_SHFT_PI_EN, HIGH);
  Serial.println("Pi's IMU's level shifter enabled.");

  // initialize tumbling detection output pins
  /*pinMode(PR_en, OUTPUT);
  pinMode(IMU_en, OUTPUT);
  pinMode(Pi5_en, OUTPUT);
  pinMode(SDR_en, OUTPUT);
  pinMode(DCDC_EN, OUTPUT); 
  pinMode(LVL_SHFT_EN, OUTPUT); 
  // PI's IMU i2c level shifter 

  // initialize current sensor voltage 
  pinMode(PI_IMU, INPUT);
  pinMode(ESP_IMU, INPUT);
  //pinMode(PI_CURRENT, INPUT);

  // enable DCDC converter & I2C level shifters 
  digitalWrite(DCDC_EN, LOW);
  Serial.println("DCDC NOT Enabled.");
  digitalWrite(LVL_SHFT_EN, LOW);
  Serial.println("Level Shifter NOT Enabled.");

  // enable devices 
  digitalWrite(IMU_en, LOW);
  Serial.println("IMU: OFF");
  digitalWrite(Pi5_en, LOW);
  Serial.println("Pi: OFF");
  digitalWrite(SDR_en, HIGH);
  Serial.println("SDR: ON");
  digitalWrite(PR_en, LOW);
  Serial.println("Photoresistor: OFF");

  pinMode(PI_SDA, OUTPUT);
  digitalWrite(PI_SDA, LOW);
  Serial.println("Pi SDA: LOW");

  

  float pi_imu_current;
  float *p = &pi_imu_current;
  float esp_imu_current;
  float *e = &esp_imu_current;
  
  measure_imu_currents(pi_imu_current, esp_imu_current);
  Serial.print("Pi IMU current: ");
  Serial.println(pi_imu_current);
  Serial.print("ESP IMU current: ");
  Serial.println(pi_imu_current);*/

  delay(1000);

  // init imu
  Serial.println("Adafruit ICM20948 test!");
  // Try to initialize!
  if (!icm.begin_I2C((uint8_t)0x68, &ESPBus, (int32_t)0)) {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

    Serial.println("Failed to find ICM20948 chip");
    while (!icm.begin_I2C((uint8_t)0x68, &ESPBus, (int32_t)0)) {
      Serial.print("Trying...");
      delay(5000);
    }
  }
  Serial.println("Success!!");

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
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("looping...");
  delay(1000);
}
