#include "Sensors.h"

// take average 500 times 
const int num_samples = 1000;
long int pi_imu_sensor_value = 0;
long int esp_imu_sensor_value = 0;
long int pi_sensor_value = 0;

// datasheet- 264mV/A(Typ.) sensitivity for 3.3V, 800mV/A(Typ.) sensitivity for 5V
float pi_sensitivity = 1 / 0.200; 
float pi_imu_sensitivity = 1 / 0.132;
float esp_imu_sensitivity = 1 / 0.132; // out of 5v
// out of 3.3v -> 

// Vref is zero drift value, change to MEASURED value 
//float esp_imu_Vref = 322; 
//float pi_imu_Vref = 322; // IDK IF THIS IS RIGHT!!??


String measure_imu_currents(double &pi_imu_current, double &esp_imu_current, double &pi_current) 
{
  String anomaly = "Detected:";
  // read sensor data num_samples times for accuracy
  for (int i = 0; i < num_samples; i++)
  {
    pi_imu_sensor_value += analogRead(PI_IMU_CURRENT);
    esp_imu_sensor_value += analogRead(ESP_IMU_CURRENT);
    pi_sensor_value += analogRead(PI_CURRENT);

    // TODO: remove? put in a task?? 
    delay(2); // 2ms before next loop 
  }

  // take average 
  pi_imu_sensor_value = pi_imu_sensor_value / num_samples;
  esp_imu_sensor_value = esp_imu_sensor_value / num_samples;
  pi_sensor_value = pi_sensor_value / num_samples;

  // unit voltage = referance voltage / bits per volt * 1000; (mV)
  float pi_imu_unit_value = PI_IMU_REF_VAL/4095.0;
  float esp_imu_unit_value = ESP_IMU_REF_VAL/4095.0; 
  float pi_unit_value = PI_REF_VAL/4095.0;

  /*Serial.print("Pi's IMU's # bits:  ");
  Serial.println(pi_imu_sensor_value);
  Serial.print("ESP's IMU's # bits: ");
  Serial.println(esp_imu_sensor_value);
  Serial.print("Pi's # bits:        ");
  Serial.println(pi_sensor_value);*/

   // only 4 significant digits
   // highest current = 10A -> 10.00A precise 
  float pi_imu_voltage =  (3.3/4095.0)*pi_imu_sensor_value;
  float esp_imu_voltage = (3.3/4095.0)*esp_imu_sensor_value;
  float pi_voltage =      ((5.0*(2.0/3.0))/4095.0)*pi_sensor_value;

  // check Pi IMU current voltage range
  if (pi_imu_voltage > 1.53) {
    anomaly += "Pi IMU (positive i spike), ";
    Serial.println("Pi IMU positive current spike.");
  } else if (pi_imu_voltage < 1.49) {
    anomaly += "Pi IMU (negative i spike), ";
    Serial.println("Pi IMU negative current spike.");
  } else {
    Serial.println("Pi IMU current nominal.");
  }

  // check ESP IMU current voltage range
  if (esp_imu_voltage > 1.53) {
    anomaly += "ESP IMU (positive i spike), ";
    Serial.println("ESP IMU positive current spike.");
  } else if (pi_imu_voltage < 1.49) {
    anomaly += "ESP IMU (negative i spike), ";
    Serial.println("ESP IMU negative current spike.");
  } else {
    Serial.println("ESP IMU current nominal.");
  }

  // check Pi current range
  if (pi_current > 0.6) {
    anomaly += "Pi (positive i spike), ";
  } else if (pi_current < -0.1) {
    anomaly += "Pi (negative i spike), ";
  }

  // IMUs: 1.65 = 0A 
  // Pi:   0.33 = 0A

  // when no load, Vref=initialValue
  Serial.print("Pi's IMU's voltage value:  ");
  Serial.print(pi_imu_voltage);
  Serial.println(" V"); 
  Serial.print("ESP's IMU's voltage value: ");
  Serial.print(esp_imu_voltage);
  Serial.println(" V"); 
  Serial.print("Pi's voltage value:        ");
  Serial.print(pi_voltage);
  Serial.println(" V");

  // control sensitivity
  pi_imu_voltage = (int)(pi_imu_voltage * 100 + .5);
  pi_imu_voltage = (float)pi_imu_voltage / 100;
  esp_imu_voltage = (int)(esp_imu_voltage * 100 + .5);
  esp_imu_voltage = (float)esp_imu_voltage / 100;
  pi_voltage = (int)(pi_voltage * 100 + .5);
  pi_voltage = (float)pi_voltage / 100;
  
  // calculate the corresponding current, store in values passed by reference 
  pi_imu_current = (pi_imu_voltage - 1.51)*(1 / 0.132);
  esp_imu_current = (esp_imu_voltage - 1.51)*(1 / 0.132);

  // (2.5*2)/3 = 1.67 = zero current voltage for pi 
  pi_current = (pi_voltage - 1.09)*pi_sensitivity*(3.3/5.0); 

  Serial.print("Pi's IMU's current:  ");
  Serial.print(pi_imu_current);
  Serial.println(" A");
  Serial.print("ESP's IMU's current: ");
  Serial.print(esp_imu_current);
  Serial.println(" A");
  Serial.print("Pi's current:        ");
  Serial.print(pi_current);
  Serial.println(" A");
  Serial.print("\n");

  // print voltage (mV) on pin corresponding to current 
  /*pi_imu_voltage = pi_imu_unit_value * pi_imu_sensor_value - PI_IMU_REF_VAL;
  esp_imu_voltage = esp_imu_unit_value * esp_imu_sensor_value - ESP_IMU_REF_VAL;

  Serial.print("Pi's IMU's voltage <-> current:  ");
  Serial.print(pi_imu_voltage); // unit volts per bit ? 
  Serial.println("mV");

  Serial.print("ESP's IMU's voltage <-> current: ");
  Serial.print(esp_imu_voltage); // unit volts per bit ? 
  Serial.println("mV");*/

  // print current (mA)
  int SDRv = analogRead(SDR_in);
  if ((SDRv < 1)&&(currentState == downlink)) {
    anomaly += " SDR,";
  }
  
  return anomaly;
}
