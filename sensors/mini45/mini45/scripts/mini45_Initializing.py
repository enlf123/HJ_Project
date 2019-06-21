#!/usr/bin/env python

import time
import serial

ser = serial.Serial(port = '/dev/ttyUSB0',baudrate=115200)
# ser.open()
ser.write('CB\r') # Communication for ASCII
time.sleep(0.01)
response = ser.readline()
if response == 'CB\r\n':
    response = ser.readline()
    print(response)
    response = ser.readline()
else:
    print('set your baudrate at 115200')
# ser.write('RS\r')

ser.write('CD A\r') # Communication for ASCII
time.sleep(0.01)
response = ser.readline()
if response == '>CD A\r\n':
    response = ser.readline()
    print('Communication with ASCII')
else:
    print('Communication with ASCII error, try again')

ser.write('CV 04\r') # Mode selection of Resolved force/torque
time.sleep(0.01)
response = ser.readline()
if response == '>CV 04\r\n':
    response = ser.readline()
    print('Only Resolved Z axis torque(unit : count)')
else:
    print('mode selection of resolved torque error, try again')



ser.write('CD R\r') # Communication for Resolved force/torque
time.sleep(0.01)
response = ser.readline()
if response == '>CD R\r\n':
    response = ser.readline()
    print('Communication with Resolved torque(unit : count)')
else:
    print('Communication with Resolved torque error, try again')

ser.write('SA 16\r') # Moving average fillter 0 2 4 8 16 32 64 128
time.sleep(0.01)
response = ser.readline()
if response == '>SA 16\r\n':
    response = ser.readline()

    print('Moving average fillter : 16')
else:
    print('Moving average fillter error, try again')


ser.write('ST 1\r') # Temperature compensation
time.sleep(0.01)
response = ser.readline()
response = ser.readline()

ser.write('SZ\r') # Sensor Zero bias
time.sleep(0.01)
response = ser.readline()
response = ser.readline()

for i in range(5):# remove bias  in 10
    ser.write('QR\r') # fill the buffer
    time.sleep(0.01)
    response = ser.readline()
    response = ser.readline()
    response = ser.readline()
    ser.write('SB\r') # remove bias
    time.sleep(0.01)
    response = ser.readline()
    response = ser.readline()
    print('loading for removing bias......')

print('Complete for removing bias')

#

print('Complete for Initializing.............')


