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
3. Plug your computer in to the S3 via the USB port & hit upload 

### Connections 
I2C bus A: the bus controlled by the Pi 
```
SDA: Pi GPIO 02 <--> ESP32 S3 GPIO 47 <--> ESP32 kitboard 22
SCL: Pi GPIO 03 <--> ESP32 S3 GPIO 21 <--> ESP32 kitboard 21
```

Other pins being used 
* GPIO 48 --> "device" with ID 0
* GPIO 06 --> "device" with ID 1
* GPIO 05 --> analog input reading the current sensor 

I2C bus B: the bus controlled by the S3 (proposed)
* SDA: GPIO 10 ?
* SCL: GPIO 11 ?
(it's just gotta be a pin that's unused & GPIO on the pinout see below)

### Pi Pinout
![pasted image 0](https://user-images.githubusercontent.com/65368903/218164949-856f9e2f-7397-4d0b-ae98-d11cedf292d7.png)

### ESP32-S3 Pinout
![ESP32-S3_DevKitC-1_pinlayout_v1 1](https://user-images.githubusercontent.com/65368903/221910972-081a4623-f7eb-41a3-b11f-dafa29ed31ba.jpg)
