#echo "Adding CAN network"
#sudo ip link add dev can0 type can
echo "Setting CAN network bitrate and queue length"
sudo ip link set can0 type can bitrate 1000000
sudo ifconfig can0 txqueuelen 1000
echo "Enabling the CAN network"
sudo ip link set down can0
sudo ip link set up can0