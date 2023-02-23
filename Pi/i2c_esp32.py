#!/usr/bin/env python

# SDA1 GPIO2 pin 3
# SCL1 GPIO3 pin 5

#    J8
# 1     2
# SDA   5V
# SCL   GND

# connect GPIO2 on Pi to 22 on ESP32 (SDA)
# connect GPIO3 on Pi to 21 on ESP32 (SDL)

# ls /dev/*i2c* results in /dev/i2c-1

import array
from smbus2 import SMBus
 

def main():
    # i2c parameters
    addr = 0x8 # ESP32 address
    offset = 0
    bus = SMBus(1) # 1 = /dev/ic2-1

    print ("Enter message to send to ESP32:")
    while True:
        msg = input(">  ")

        # normalize msg to 32 bytes
        msg = msg + '\0'
        if len(msg) > 32:
            msg = msg[0:32]
        elif len(msg) < 32:
            while len(msg) < 32:
                msg = msg + '\n'
        print(f'msg: {msg}')
        print(f'len: {len(msg)}')
        # convert to byte array
        byte_msg = bytearray()
        byte_msg.extend(map(ord, msg))
        print(f'bytes: {byte_msg}')
        print(f'len:   {len(byte_msg)}')
        bus.write_i2c_block_data(addr, offset, byte_msg)
        print('Done sending.')

if __name__ == '__main__':
    main()
