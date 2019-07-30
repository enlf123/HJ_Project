#!/usr/bin/env python

import time
import serial

ser = serial.Serial(port = '/dev/ttyUSB0',baudrate=115200)
# ser.port = "/dev/ttyUSB0"
# ser.baudrate = 9600
#ser.open()
i=1

while 1:
    #print("response")
    response = ser.readline()
    print(response)
