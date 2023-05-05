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
    def send_msg(self, msg, verbose=False):
        if len(msg) > 32:
            print('ERROR: msg is > 32B, use send_long_msg >:(')
            return False
        
        # normalize to 32B
        while len(msg) < 32:
            msg = msg + '\n'

        if verbose:
            print(f'Sending 32B message: {msg.strip()}')
            print(f'Length of message:   {len(msg)}')
            print('')

        # convert to byte array
        byte_msg = bytearray()
        byte_msg.extend(map(ord, msg))
        if verbose:
            print(f'bytes: {byte_msg}')
            print(f'len:   {len(byte_msg)}')
            print('')

        # write bytes
        self.bus.write_i2c_block_data(self.addr, self.offset, byte_msg)
        if verbose:
            print('Done sending.')
        return True

    # request status report from the ESP32
    def get_anomaly_report(self):
        # send status report request to esp
        self.send_msg('4:0')
        msg = '0:' + '\0'
        if not self.send_msg(msg):
            return ''
        
        msg = ''
        # read response from esp
        byte = self.bus.read_byte(self.addr)
        msg += chr(byte)
        #print(f'Skipping: {byte} -> {hex(byte)} -> {chr(byte)}')
        time.sleep(0.25)
        while byte != 0:
            byte = self.bus.read_byte(self.addr)
            msg += chr(byte)
            #print(f'{byte} -> {hex(byte)} -> {chr(byte)}')
            time.sleep(0.25)
        msg += chr(self.bus.read_byte(self.addr))
        time.sleep(0.1)

        # remove opcode:
        #msg = msg[2:]

        return msg
    
    def get_tumbling_report(self):
        # send status report request to esp
        self.send_msg('4:3')
        msg = '0:' + '\0'
        if not self.send_msg(msg):
            return ''


        
        msg = ''
        # read response from esp
        #msg += chr(self.bus.read_byte(self.addr))
        byte = self.bus.read_byte(self.addr)
        letter = chr(byte)
        #print(f'Skipping: {byte} -> {hex(byte)} -> {chr(byte)}')
        time.sleep(0.25)
        byte = self.bus.read_byte(self.addr)
        letter = chr(byte)
        #print(f'Skipping: {byte} -> {hex(byte)} -> {chr(byte)}')
        time.sleep(0.25)
        #msg += letter
        while byte != 0:
            byte = self.bus.read_byte(self.addr)
            letter = chr(byte)
            #print(f'{byte} -> {hex(byte)} -> {chr(byte)}')
            msg += letter
            time.sleep(0.25)
        msg += chr(self.bus.read_byte(self.addr))
        time.sleep(0.1)

        # remove opcode:
        #msg = msg[2:]
        return msg


    # request message from ESP32 for SDR
    def get_msg(self):
        # opcode 4 sets request id, request id 0 is send message
        self.send_msg('4:2')
        
        # read in 8 bytes = sizeof(double) 
        msg = ''
        for i in range(0, 23):
            msg += chr(self.bus.read_byte(self.addr))
            time.sleep(0.05)
        time.sleep(0.1)

        print(msg)
        return msg

    # request current data from ESP32
    def get_current_sensor_readings(self):
        # opcode 4 sets request id, request id 1 is send sensor readings
        self.send_msg('4:1')

        # read in 8 bytes = sizeof(double) for Pi current
        status = bytearray()
        byte = self.bus.read_byte(self.addr)
        #print(f'Skipping: {byte} -> {hex(byte)} -> {chr(byte)}')
        time.sleep(0.25)
        for i in range(0, 8):
            #status.append(self.bus.read_byte(self.addr))
            byte = self.bus.read_byte(self.addr)
            status.append(byte)
            #print(f'{byte} -> {hex(byte)} -> {chr(byte)}')
            time.sleep(0.25)
        time.sleep(2)
        piIMUCurrent = struct.unpack('d', status)[0]
        
        # read in ESP's IMU's current
        status = bytearray()
        byte = self.bus.read_byte(self.addr)
        #print(f'Skipping:{byte} -> {hex(byte)} -> {chr(byte)}')
        time.sleep(0.25)
        for i in range(0, 8): # 8 bytes = sizeof(double)
            #status.append(self.bus.read_byte(self.addr))
            byte = self.bus.read_byte(self.addr)
            #print(byte)
            status.append(byte)
            #print(f'{byte} -> {hex(byte)} -> {chr(byte)}')
            time.sleep(0.25)
        time.sleep(2)
        espIMUCurrent = struct.unpack('d', status)[0]
        
        # read in Pi's IMU's current
        status = bytearray()
        byte = self.bus.read_byte(self.addr)
        #print(f'Skipping: {byte} -> {hex(byte)} -> {chr(byte)}')
        time.sleep(0.25)
        for i in range(0, 8):
            #status.append(self.bus.read_byte(self.addr))
            byte = self.bus.read_byte(self.addr)
            #print(byte)
            status.append(byte)
            #print(f'{byte} -> {hex(byte)} -> {chr(byte)}')
            time.sleep(0.25)
        time.sleep(2)
        piCurrent = struct.unpack('d', status)[0]
        
        # read in battery &
        status = bytearray()
        byte = self.bus.read_byte(self.addr)
        #print(f'Skipping: {byte} -> {hex(byte)} -> {chr(byte)}')
        time.sleep(0.25)
        for i in range(0, 8):
            status.append(self.bus.read_byte(self.addr))
            #print(f'{byte} -> {hex(byte)} -> {chr(byte)}')
            time.sleep(0.25)
        time.sleep(2)
        batteryPerc = struct.unpack('d', status)[0]

        return piIMUCurrent, espIMUCurrent, piCurrent, batteryPerc

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
    while True:
        esp = ESP32Client()
        msg = esp.get_tumbling_report()
        print(f'Tumbling report from ESP32: {msg}')
        time.sleep(2)
        msg = esp.get_anomaly_report()
        print(f'Anomaly report from ESP32:  {msg}')
        time.sleep(2)
        #piIMUCurrent, espIMUCurrent, piCurrent, batteryLevel = esp.get_current_sensor_readings()
        #print(f'Pi IMU Current:   {piIMUCurrent:.2f} A')
        #print(f'ESP IMU Current:  {espIMUCurrent:.2f} A')
        #print(f'Pi Current:       {piCurrent:.2f} A')
        #time.sleep(2)

    #while True:
        # get status report
        #msg = esp.get_tumbling_report()
        #print(f'Tumbling report from ESP32: {msg}')
        #msg = esp.get_anomaly_report()
        #print(f'Anomaly report from ESP32:  {msg}')

        # get current values
        #piIMUCurrent, espIMUCurrent, piCurrent, batteryLevel = esp.get_current_sensor_readings()
        #print(f'Pi IMU Current:   {piIMUCurrent:.2f} A')
        #print(f'ESP IMU Current:  {espIMUCurrent:.2f} A')
        #print(f'Pi Current:       {piCurrent:.2f} A')
        #print(f'Battery %:        {batteryLevel:.2f}%')
        #print()
        #esp.bus.close()
        #esp.bus = SMBus(1) # 1 = /dev/ic2-1
        #time.sleep(4)
        

if __name__ == '__main__':
    main()
