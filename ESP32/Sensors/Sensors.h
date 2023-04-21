#ifndef __Sensors__
#define __Sensors__

#include "Arduino.h"

#define PI_IMU_REF_VAL 5.0
#define ESP_IMU_REF_VAL 3.3

#define PI_IMU 39  // Pi's IMU on pin 7 = I39
#define ESP_IMU 34 // ESP's IMU is pin 9 = I34

// take average 500 times 
extern const int num_samples;
extern long int pi_imu_sensor_value;
extern long int esp_imu_sensor_value;

// datasheet- 264mV/A(Typ.) sensitivity for 3.3V, 800mV/A(Typ.) sensitivity for 5V
extern float pi_imu_sensitivity;
extern float esp_imu_sensitivity;

// Vref is zero drift value, change to MEASURED value 
//extern float esp_imu_Vref;
//extern float pi_imu_Vref;

void measure_imu_currents(float &pi_imu_current, float &esp_imu_current);

#endif
