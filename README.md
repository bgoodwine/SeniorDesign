# SeniorDesign :)
Code for Senior Design II

### Pi
1. Get on ND-Guest wifi
2. `ssh pi@razorcrest` (typical irishsat pass)
3. Test scripts in directory `bridget`

[smbus2 documentation](https://pypi.org/project/smbus2/)

### ESP32
Upload `main.cpp` with extra line `monitor_speed = 115200` in `platformio.ini` file of the project.

### I2C connections
```
SDA: Pi GPIO 02 <--> ESP32 S3 GPIO 47 <--> ESP32 kitboard 22
SCL: Pi GPIO 03 <--> ESP32 S3 GPIO 21 <--> ESP32 kitboard 21
```

### Pi Pinout
![pasted image 0](https://user-images.githubusercontent.com/65368903/218164949-856f9e2f-7397-4d0b-ae98-d11cedf292d7.png)

### ESP32-S3 Pinout
![ESP32-S3_DevKitC-1_pinlayout_v1 1](https://user-images.githubusercontent.com/65368903/221910972-081a4623-f7eb-41a3-b11f-dafa29ed31ba.jpg)
