# SeniorDesign :)
Code for Senior Design II

### Login to the Pi
1. Get on ND-Guest wifi
2. `ssh pi@192.168.10.239` (login with the normal IrishSat pi password)
3. `cd python` to get to the program `esp32.py`

The Pi code uses [smbus2](https://pypi.org/project/smbus2/).

### Setup the ESP32-S3
1. Make a new project with platformio on vscode and select `dev board ESP32-S3-DevKitC-1` ([instructions](https://hutscape.com/tutorials/blinky-platformio-esp32s3))
2. Replace default files with `main.cpp` and `platform.ini` in the `ESP32` directory
3. Go to the Libraries tab and add `Adafruit_Sensor` and `Adafruit_BNO055` to the project ([instructions](https://docs.platformio.org/en/latest/librarymanager/dependencies.html))
4. Plug your computer in to the S3 via the USB port & hit upload 

### I2C Busses
PiBus: I2C bus controlled by the Pi Zero, connected to ESP32 S3
* `SDA: ESP32 S3 GPIO 47 <--> Pi Zero GPIO 02`
* `SCL: ESP32 S3 GPIO 21 <--> Pi Zero GPIO 03`

ESPBus: I2C bus controlled by the ESP32 S3, connected to IMU
* `SDA: ESP32 S3 GPIO 38`
* `SCL: ESP32 S3 GPIO 37`

Note: ESPBus will eventually connect to the fuel gauge

### Current Pinout
```
+------------------------------------------+--------+---------------------------+
| S3 Pin                                   | S3 Pin | Usage                     |
+--------+---------------------------------+--------+---------------------------+
| 3.3V   | 3.3V power to devices           | GND    |                           |
+--------+---------------------------------+--------+---------------------------+
| 3.3V   |                                 | U0TXD  |                           |
+--------+---------------------------------+--------+---------------------------+
| RST    |                                 | U0RXD  |                           |
+--------+---------------------------------+--------+---------------------------+
| GPIO4  | Current sensor analog output    | GPIO1  |                           |
+--------+---------------------------------+--------+---------------------------+
| GPIO5                                    | GPIO2  |                           |
+--------+---------------------------------+--------+---------------------------+
| GPIO6  | Device 1 on/off                 | MTMS   |                           |
+--------+---------------------------------+--------+---------------------------+
| GPIO7  |                                 | MTDI   |                           |
+--------+---------------------------------+--------+---------------------------+
| GPIO15 | Photo resistor analog input     | MTDO   |                           |
+--------+---------------------------------+--------+---------------------------+
| GPIO16 | IMU analog input                | MTCK   |                           |
+--------+---------------------------------+--------+---------------------------+
| GPIO17 | Pi 5 analog input               | GPIO38 | ESPBus I2C SDA            |
+--------+---------------------------------+--------+---------------------------+
| GPIO18 | SDR analog input                | GPIO37 | ESPBus I2C SCL            |
+--------+---------------------------------+--------+---------------------------+
| GPIO8  | Datalink digital input (unused) | GPIO36 |                           |
+--------+---------------------------------+--------+---------------------------+
| GPIO3  |                                 | GPIO35 |                           |
+--------+---------------------------------+--------+---------------------------+
| GPIO46 |                                 | GPIO0  |                           |
+--------+---------------------------------+--------+---------------------------+
| GPIO9  | Photo resistor analog output    | GPIO45 |                           |
+--------+---------------------------------+--------+---------------------------+
| GPIO10 | IMU analog output               | GPIO48 | Device 0 on/off           |
+--------+---------------------------------+--------+---------------------------+
| GPIO11 | Pi 5 analog output              | GPIO47 | PiBus I2C SDA             |
+--------+---------------------------------+--------+---------------------------+
| GPIO12 | SDR analog output               | GPIO21 | PiBus I2C SCL             |
+--------+---------------------------------+--------+---------------------------+
| GPIO13 |                                 | USB_D+ |                           |
+--------+---------------------------------+--------+---------------------------+
| GPIO14 |                                 | USB_D- |                           |
+--------+---------------------------------+--------+---------------------------+
| 5V0    |                                 | GND    |                           |
+--------+---------------------------------+--------+---------------------------+
| GND    |                                 | GND    | GND connection to devices |
+--------+---------------------------------+--------+---------------------------+
```

### Pi Pinout
![pasted image 0](https://user-images.githubusercontent.com/65368903/218164949-856f9e2f-7397-4d0b-ae98-d11cedf292d7.png)

### ESP32-S3 Pinout
![ESP32-S3_DevKitC-1_pinlayout_v1 1](https://user-images.githubusercontent.com/65368903/221910972-081a4623-f7eb-41a3-b11f-dafa29ed31ba.jpg)
