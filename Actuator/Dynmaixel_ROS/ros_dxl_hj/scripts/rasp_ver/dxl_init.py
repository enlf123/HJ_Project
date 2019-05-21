#!/usr/bin/env python

import rospy
import time
from dynamixel_sdk import *
from std_msgs.msg import Int32

# Control table address
ADDR_PRO_BAUD_RATE	    	= 8
ADDR_PRO_RETURN_TIME	    = 9
ADDR_PRO_OPERATING_MODE	    = 11
ADDR_PRO_HOMING_OFFSET 	    = 20
ADDR_PRO_MAX_POS 	    	= 48
ADDR_PRO_MIN_POS 	    	= 52
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
BAUDRATE                    = 3000000             # Dynamixel default baudrate : 115200
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
BAUD_RATE_SET		    	= 5
OPERATING_MODE		    	= 3
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0          	# Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4095            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 1                # Dynamixel moving status threshold
DXL_RETURN_DELAY_TIME       = 25		# Dynamixel Return delay time (0*2us)

# index = 0
# dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

def dxl_init():
    # Open Port
    if portHandler.openPort():
        print "Succeeded to open the port"
    else:
        print "Failed to open the port"
        quit()

    # Set Port Baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print "Succeeded to change the Baudrate"
    else:
        print "Failed to change the Baudrate"
        quit()


    #unable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "Dynamixel is disable to torque"

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_BAUD_RATE, BAUD_RATE_SET)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "Baudrate of Dynamixel is 1000000" 

    #Set operating mode
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_OPERATING_MODE, OPERATING_MODE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "Set Position Mode"    

    #Return Delay Time 20us
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_RETURN_TIME, DXL_RETURN_DELAY_TIME)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "Return delay time of Dynamixel has been successfully changed : %02d" %(DXL_RETURN_DELAY_TIME*2)

    #DXL_MAXIMUM_POSITION_VALUE
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_MAX_POS, DXL_MAXIMUM_POSITION_VALUE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "MAXIMUM POS of Dynamixel has been successfully changed : %04d" %(DXL_MAXIMUM_POSITION_VALUE)

    #DXL_MINIMUM_POSITION_VALUE
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_MIN_POS, DXL_MINIMUM_POSITION_VALUE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "MINIMUM POS of Dynamixel has been successfully changed : %04d" %(DXL_MINIMUM_POSITION_VALUE)

    #Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "Dynamixel is able to torque"


    

if __name__ == '__main__':
    dxl_init()
    portHandler.closePort()

