Raspberry Pi CAN communication
sudo slcand -o -c -f -s4 /dev/ttyUSB0 slcan0

sudo ifconfig slcan0 up

CAN Communication monitor
candump slcan0
