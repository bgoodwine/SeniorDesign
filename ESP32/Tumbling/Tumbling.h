#ifndef __Tumbling__
#define __Tumbling__

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include "Adafruit_MAX1704X.h"
#include <SPI.h> // needed for BNO055 library
#include <stdint.h> // needed for int8_t datatype 
//#include <utility/imumaths.h>
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

#define PR_in  38 // TODO: we'll need to map to current instead
#define SDR_in 26

#define DCDC_EN         13
#define LVL_SHFT_EN     4
#define LVL_SHFT_PI_EN  21

#define PI_IMU_en  23
#define PR_en      32
#define IMU_en     27
#define Pi5_en     18
#define SDR_en     15

#define battAnomaly  1
#define IMUAnomaly   2
#define SDRAnomaly   3
#define Pi5Anomaly   4
#define noAnomaly    0


extern TwoWire ESPBus;
extern Adafruit_ICM20948 icm;
extern Adafruit_MAX17048 maxlipo;

// tumbling external global state variables
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


// Function declarations
int8_t temperature_read();
String sendReport(int anomaly, float oldBatteryLevel, float batteryLevel);
void tumblingDetection(float (& outputsArray)[3]);
void updateState(int currentState, int oldCurrentState);
int checkAnomalies(int currentState);
int dayCycleCheck();
int downlinkCheck();
int checkState(int currentState, int oldCurrentState, float batteryLevel);
int8_t temperature_read();

#endif
