#ifndef __PiComms__
#define __PiComms__

#include "Arduino.h"
#include "Tumbling.h"
#include <Wire.h>

// i2c definitions 
#define PI_SDA   47 // Pi Zero SDA = GPIO2
#define PI_SCL   21 // Pi Zero SCL = GPIO3
#define ESP_ADDR 0x08 // ESP32 address
#define MAXINDEX 7
#define ESP_SDA  38 // ESP S3's I2C bus
#define ESP_SCL  37 

// from Tumbling
#define PR_en  15
#define IMU_en 10
#define Pi5_en 11
#define SDR_en 12


// device on/off pin definitions 
#define DEVICE_0   48
#define DEVICE_1   6
#define CURRENTPIN 34 // TODO: this is IMU, add Pi

extern TwoWire PiBus;

typedef union u{
  double d;
  char bytes[sizeof(double)];
}u;

typedef union f{
  float f;
  char bytes[sizeof(float)];
}f;

extern double piIMUCurrent;
extern double piCurrent;
extern double espIMUCurrent;

extern int current_request;
extern float batteryLevel;
extern String current_tumbling_report;
extern String current_anomaly_report;
extern int current_tumbling_state;

double readcurrent(int num_readings);
void send_sdr_msg();
void send_current_sensor_readings();
void requestEvent();
void send_tumbling_report();
void send_anomaly_report();
void restart_device(int device_id);
void shutdown_device(int device_id);
void misc_msg(char *data);
void receiveEvent(int bytes_to_read);

#endif

