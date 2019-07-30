#!/usr/bin/env python

import rospy
from dynamixel_sdk import *
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32


# Control table address
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
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB1'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0          # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4095            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 1                # Dynamixel moving status threshold

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

    #Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "Dynamixel has been successfully connected"


def dxl_poscon(goal_pos):
    dxl_goal_position = 0+goal_pos

    # Write Goal Position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    pub = rospy.Publisher('dxl_position', Int32, queue_size=1)
#    rospy.init_node('dxl_pos_read', anonymous=True)
    rate = rospy.Rate(100)

#    while not rospy.is_shutdown():
    # Read Present Position
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)

    print "GoalPos:%03d PresPos:%03d" %(dxl_goal_position, dxl_present_position)

    rospy.loginfo(dxl_present_position)
    pub.publish(dxl_present_position)
    rate.sleep()

#        if not abs(dxl_goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
#            break




def callback(msgData):
    goal_pos = int(msgData.data)
    dxl_poscon(goal_pos)

def dxl_con():
    rospy.init_node('dxl_con', anonymous=True)
    rospy.Subscriber('dxl_desired_pos', Float32, callback)
    rospy.spin()



if __name__ == '__main__':
    dxl_init()
    dxl_con()


    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error!= 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)

    # Close Port
    portHandler.closePort()
