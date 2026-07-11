#! /bin/bash
#echo "Adding CAN network"
#sudo ip link add dev can1 type can

sudo ip link set down can1
echo "Setting CAN network bitrate and queue length"
sudo ip link set can1 type can bitrate 500000
sudo ifconfig can1 txqueuelen 1000
echo "Enabling the CAN network"
sudo ip link set down can1
sudo ip link set up can1

sudo gpioset gpiochip2 9=0
sudo gpioset gpiochip2 8=0
