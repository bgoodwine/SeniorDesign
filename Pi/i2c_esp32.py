#!/usr/bin/env python

# SDA1 GPIO2
# SCL1 GPIO3

#    J8
# 1       2
# GPIO2   5V
# GPIO3   GND

# connect GPIO2 on Pi to 22 on ESP32 (SDA)
# connect GPIO3 on Pi to 21 on ESP32 (SDL)

import array
#from smbus import SMBus
from smbus2 import SMBus

 
addr = 0x8 # ESP32 address
offset = 0
# ls /dev/*i2c* results in /dev/i2c-1
bus = SMBus(1) # indicates /dev/ic2-1
 
 
print ("Enter message to send to ESP32:")
while True:
    msg = input(">  ")
    # normalize msg to 256 bytes
    if len(msg) > 128:
        msg = msg[0:128]
    elif len(msg) < 32:
        while len(msg) < 32:
            msg = msg + '0'
    print(f'msg: {msg}')
    print(f'len: {len(msg)}')
    # convert to byte array
    byte_msg = bytearray()
    byte_msg.extend(map(ord, msg))
    print(f'bytes: {byte_msg}')
    print(f'len:   {len(byte_msg)}')
    bus.write_i2c_block_data(addr, offset, byte_msg)
    print('Done sending.')

        #bus.write_byte(addr, 0x1) # switch it on
