#ifndef __PiComms__
#define __PiComms__

#include "Arduino.h"
#include <Wire.h>

// i2c definitions 
#define PI_SDA 47 // Pi Zero SDA = GPIO2
#define PI_SCL 21 // Pi Zero SCL = GPIO3
#define ESP_ADDR 0x08 // ESP32 address
#define MAXINDEX 7
#define ESP_SDA 38 // ESP controlled I2C bus
#define ESP_SCL 37 // ESP controlled I2C bus

// from Tumbling
#define PR_en 9
#define IMU_en 10
#define Pi5_en 11
#define SDR_en 12


// device on/off pin definitions 
#define DEVICE_0 48
#define DEVICE_1 6
#define CURRENTPIN 4

extern TwoWire PiBus;

typedef union u{
  double d;
  char bytes[sizeof(double)];
}u;

extern u currentPi;
extern u currentIMU;
extern int current_request;

double readcurrent(int num_readings);
void send_sdr_msg();
void send_current();
void requestEvent();
void checkin();
void restart_device(int device_id);
void shutdown_device(int device_id);
void misc_msg(char *data);
void receiveEvent(int bytes_to_read);

#endif

