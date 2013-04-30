#!/bin/bash

RFCOMM='0'
device="/dev/rfcomm$RFCOMM"
BT_ADDRESS='10:00:E8:6C:F0:4A'
EXEC='rosrun asebaros asebaros'

# bind to rfcomm
if [ -e $device ]; then 
        echo "$device exists"
else
        echo "Binding to $device"
        sudo rfcomm bind $RFCOMM $BT_ADDRESS
fi

echo "Connecting to target"
$EXEC "ser:device=$device;fc=hard;baud=921600"

