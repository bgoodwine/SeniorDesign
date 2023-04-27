#include "Sensors.h"

// take average 500 times 
const int num_samples = 500;
long int pi_imu_sensor_value = 0;
long int esp_imu_sensor_value = 0;
long int pi_sensor_value = 0;

// datasheet- 264mV/A(Typ.) sensitivity for 3.3V, 800mV/A(Typ.) sensitivity for 5V
float pi_sensitivity = 1000.0 / 264.0; 
float pi_imu_sensitivity = 1000.0 / 800.0;
float esp_imu_sensitivity = 1000.0 / 800.0;

// Vref is zero drift value, change to MEASURED value 
//float esp_imu_Vref = 322; 
//float pi_imu_Vref = 322; // IDK IF THIS IS RIGHT!!??


void measure_imu_currents(double &pi_imu_current, double &esp_imu_current, double &pi_current) 
{
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
  float pi_imu_unit_value = PI_IMU_REF_VAL/1024.0;
  float esp_imu_unit_value = ESP_IMU_REF_VAL/1024.0; 
  float pi_unit_value = PI_REF_VAL/1024.0;

  Serial.print("Pi's IMU's # bits:  ");
  Serial.println(pi_imu_sensor_value/1024.0);
  Serial.print("ESP's IMU's # bits: ");
   Serial.println(esp_imu_sensor_value/1024.0);
  Serial.print("Pi's # bits:        ");
   Serial.println(pi_sensor_value/1024.0);

   // voltage = unit voltage * sensor value (mV)
  float pi_imu_voltage = pi_imu_unit_value*pi_imu_sensor_value;
  float esp_imu_voltage = esp_imu_unit_value*esp_imu_sensor_value;
  float pi_voltage = pi_unit_value*pi_sensor_value;


  // when no load, Vref=initialValue
  Serial.print("Pi's IMU's initial voltage value:  ");
  Serial.print(pi_imu_voltage);
  Serial.println(" V"); 
  Serial.print("ESP's IMU's initial voltage value: ");
  Serial.print(esp_imu_voltage);
  Serial.println(" V"); 
  Serial.print("Pi's initial voltage value:        ");
  Serial.print(pi_voltage);
  Serial.println(" V");
  
  // calculate the corresponding current, store in values passed by reference 
  pi_imu_current = (pi_imu_voltage - PI_IMU_REF_VAL)*pi_imu_sensitivity;
  esp_imu_current = (esp_imu_voltage - ESP_IMU_REF_VAL)*esp_imu_sensitivity;
  pi_current = (pi_voltage - PI_REF_VAL)*pi_sensitivity; 

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
  Serial.print("Pi's IMU's current:  ");
  Serial.print(pi_imu_current);
  Serial.println("mA");
  Serial.print("ESP's IMU's current: ");
  Serial.print(esp_imu_current);
  Serial.println("mA");
  Serial.print("Pi's current:        ");
  Serial.print(pi_current);
  Serial.println("mA");
  Serial.print("\n");

}
