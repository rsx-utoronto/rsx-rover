#!/bin/bash

#Depends on busybox to write these values to some register to make the GPIO pins into FSIO for can
sudo busybox devmem 0x0c303000 32 0x0000C400
sudo busybox devmem 0x0c303008 32 0x0000C458
sudo busybox devmem 0x0c303010 32 0x0000C400
sudo busybox devmem 0x0c303018 32 0x0000C458

#Scans the jetson IO for new interfaces
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

#set it down
sudo ip link set down can0

#Set it up
sudo ip link set can0 type can bitrate 500000 dbitrate 2000000 berr-reporting on fd on
sudo ip link set up can0

#Make queuelength 1000 (required for many FRC stuff)
sudo ifconfig can0 txqueuelen 10000

exit 0

