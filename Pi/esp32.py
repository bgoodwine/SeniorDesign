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
import struct
import time
from smbus2 import SMBus

class ESP32Client:
    def __init__(self):
        # initialize i2c parameters
        self.addr = 0x8 # ESP32 address
        self.offset = 0
        self.bus = SMBus(1) # 1 = /dev/ic2-1
        self.num_current_bytes = 8 # size of a double

    # send one 32B message to Pi
    def send_msg(self, msg):
        if len(msg) > 32:
            print('ERROR: msg is > 32B, use send_long_msg >:(')
            return

        # normalize to 32B
        while len(msg) < 32:
            msg = msg + '\n'

        print(f'Sending 32B message: {msg.strip()}')
        print(f'Length of message:   {len(msg)}')
        print('')

        # convert to byte array
        byte_msg = bytearray()
        byte_msg.extend(map(ord, msg))
        print(f'bytes: {byte_msg}')
        print(f'len:   {len(byte_msg)}')
        print('')

        # write bytes
        self.bus.write_i2c_block_data(self.addr, self.offset, byte_msg)
        print('Done sending.')

    # request status report from the ESP32
    def status_report(self):
        print(f'Sending request for status report.')
        msg = '0:' + '\0'
        self.send_msg(msg)

    # request current data from ESP32
    def get_current(self):
        self.send_msg('5:')

        # read in 8 bytes = sizeof(double)
        status = bytearray()
        for i in range(0, self.num_current_bytes):
            status.append(self.bus.read_byte(self.addr))
            time.sleep(0.05)
        time.sleep(0.1)

        current = struct.unpack('d', status)[0]
        print(current)
        return current

    def restart_sensor(self, sensor_id):
        print(f'Sending request to restart sensor {sensor_id}')
        msg = '1:' + str(sensor_id) + '\0'
        self.send_msg(msg)

    def shutdown_sensor(self, sensor_id):
        print(f'Sending request to shutdown sensor {sensor_id}')
        msg = '2:' + str(sensor_id) + '\0'
        self.send_msg(msg)

    def misc_msg(self, msg):
        print(f'Sending misc. message to ESP32: {msg}')
        opcode = '3'
        msg = '3:' + msg + '\0'
        self.send_msg(msg)

def main():
    esp = ESP32Client()
    print("Enter command to send to ESP32:")

    while True:
        msg = input(">  ")
        if msg == '0':
            esp.status_report()
        elif msg.startswith('1'):
            sensor_id = msg.split(':')[-1]
            esp.restart_sensor(sensor_id)
        elif msg.startswith('2'):
            sensor_id = msg.split(':')[-1]
            esp.shutdown_sensor(sensor_id)
        elif msg.startswith('4') or msg == 'current':
            current = esp.get_current()
            print(f'Current: {current}')
        else:
            esp.misc_msg(msg)

if __name__ == '__main__':
    main()
