#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import time
import serial
import spidev
import time
import os
from std_msgs.msg import Float32

ch = 0 #choose channel 0~7 (MCP3208)
avg_num = 8
spi = spidev.SpiDev()
spi.open(0,0) # open(bus, device)
spi.max_speed_hz = 1000000


def magnetic_read():
    pub = rospy.Publisher('magnetic_angle', Float32, queue_size=1)
    rospy.init_node('magnetic_read', anonymous=True)
    rate = rospy.Rate(200) # 100hz
    while not rospy.is_shutdown():
	value = avg_filter(ch)
        rospy.loginfo(value)
        pub.publish(value)
        rate.sleep()

def ADC2ANG(adc):
	ang = adc*360/4095.0
	return ang
	
def avg_filter(ch):
	pre_avg = 0
	for i in range(avg_num):
		ADC = ReadAdc(ch)
		value = ADC2ANG(ADC)
		avg_value = ((avg_num-1.0)/avg_num)*pre_avg + 1.0/avg_num*value
		pre_avg = float(avg_value)
	return avg_value
	
	
def ReadAdc(chNum):

	if chNum > 7 or chNum < 0:
		return -1
	adc = spi.xfer2([ 6 | (chNum&4) >> 2, (chNum&3)<<6, 0])
	data = ((adc[1]&15) << 8) + adc[2]
	return data

if __name__ == '__main__':
    try:
        magnetic_read()
    except rospy.ROSInterruptException:
        pass
