#!/usr/bin/env python

# SDA1 GPIO2
# SCL1 GPIO3

#    J8
# 1       2
# GPIO2   5V
# GPIO3   GND

# connect GPIO2 on Pi to 22 on ESP32 (SDA)
# connect GPIO3 on Pi to 21 on ESP32 (SDL)

from smbus import SMBus
 
addr = 0x8 # bus address
# ls /dev/*i2c* results in /dev/i2c-1
bus = SMBus(1) # indicates /dev/ic2-1
 
numb = 1
 
print ("Enter 1 for ON or 0 for OFF")
while numb == 1:
    ledstate = input(">>>>   ")
    if ledstate == "1":
        bus.write_byte(addr, 0x1) # switch it on
    elif ledstate == "0":
        bus.write_byte(addr, 0x0) # switch it on
    else:
        numb = 0

