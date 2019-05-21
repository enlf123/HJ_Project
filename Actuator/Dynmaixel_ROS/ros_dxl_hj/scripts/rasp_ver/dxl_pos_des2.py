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
import math
from dynamixel_sdk import *
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32

# Control table address
ADDR_PRO_RETURN_TIME	    = 9
ADDR_PRO_OPERATING_MODE	    = 11
ADDR_PRO_HOMING_OFFSET 	    = 20
ADDR_PRO_MAX_POS 	    = 48
ADDR_PRO_MIN_POS 	    = 52
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_PROFILE_VELOCITY   = 112
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4
LEN_PRO_PROFILE_VELOCITY    = 4


# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 2               # Dynamixel#1 ID : 2
BAUDRATE                    = 3000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

# ** setserial /dev/ttyUSB0 low_latency - if you want more communication speed, write this in terminal   https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/80


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)



hz = 1./100 
peak = 10/hz
index = 0
cnt2 = 0
def des_pos_trajectory(cnt):
    global index
    global peak
    global cnt2
    if cnt < 10/hz :
        dxl_goal_position = 2
    else :
        if index == 0 :
            dxl_goal_position = 2+int((cnt-peak)/0.25)
            if dxl_goal_position > 4095:
                dxl_goal_position = 4095
                if cnt2 > 100 :
                    peak = cnt
                    cnt2= 0
                    index = 1
                cnt2 = cnt2+1
        else :
            dxl_goal_position = 4095-int((cnt-peak)/0.25)                
            if dxl_goal_position < 2:
                dxl_goal_position = 2
                if cnt2 > 100 :
                    peak = cnt
                    cnt2= 0
                    index = 0                
                cnt2 = cnt2+1
#    if cnt < 5/hz :
#        dxl_goal_position = int(235.0/360.0*4095-2)
#    else :
#        if index == 0 :
#            dxl_goal_position = int(235.0/360.0*4095-2)-300+int((cnt-peak)/0.5)
#            if dxl_goal_position > int(235.0/360.0*4095-2)+300:
#                dxl_goal_position = int(235.0/360.0*4095-2)+300
#                peak = cnt
#                index = 1
#        else :
#            dxl_goal_position = int(235.0/360.0*4095-2)+300-int((cnt-peak)/0.5)                
#            if dxl_goal_position < int(235.0/360.0*4095-2)-300:
#                dxl_goal_position = int(235.0/360.0*4095-2)-300
#                peak = cnt
#                index = 0       
            
    return dxl_goal_position

def dxl_pos_read():
    portHandler.openPort()
    portHandler.setBaudRate(BAUDRATE)

    pub = rospy.Publisher('dxl_pos', Int32, queue_size=1)
    rospy.init_node('dxl_pos_read', anonymous=True)
  
    #rate = rospy.Rate(410) # 100hz
    pre = time.time()
    cnt = 0.0;
   # while not rospy.is_shutdown():
    while 1:
        cur = time.time()
        if (cur-pre)>hz:
            dxl_goal_position = des_pos_trajectory(cnt)
#        print("%s" % dxl_goal_position)
    # Write Goal Position
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position)
        #if dxl_comm_result != COMM_SUCCESS:
        #    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        #elif dxl_error != 0:
        #    print("%s" % packetHandler.getRxPacketError(dxl_error))
       
        
            dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
        #if dxl_comm_result != COMM_SUCCESS:
        #    print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
        #elif dxl_error != 0:
        #    print "%s" % packetHandler.getRxPacketError(dxl_error)
#        rospy.loginfo(dxl_present_position)
#        print(dxl_present_position)
            pub.publish(dxl_present_position)
#        portHandler.closePort()
        #rate.sleep()
            cnt = cnt+1.0
#            print(1./(cur-pre))
            pre = cur
            


if __name__ == '__main__':
    try:
        dxl_pos_read()
    except rospy.ROSInterruptException:
        pass
