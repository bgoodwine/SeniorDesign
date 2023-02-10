# SeniorDesign
Code for Senior Design II

### Pi
1. Get on ND-Guest wifi
2. `ssh pi@razorcrest` (typical irishsat pass)
3. Test scripts in directory `bridget`

### ESP32
Upload `main.cpp` with extra line `monitor_speed = 115200` in `platformio.ini` file of the project.

### I2C connections
* SDA: Pi GPIO2 --> ESP32 22
* SCL: Pi GPIO3 --> ESP32 21

![pasted image 0](https://user-images.githubusercontent.com/65368903/218164949-856f9e2f-7397-4d0b-ae98-d11cedf292d7.png)
