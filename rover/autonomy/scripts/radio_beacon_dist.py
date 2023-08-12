#!/usr/bin/env python3
import rospy
#from bluepy.btle import Scanner, ScanEntry
import math
from std_msgs.msg import Int16


def main():
    """
    How to get the rssi (signal strength) from the bluetooth beacon.
    - open a terminal and input 'bluetoothctl scan on' it will output whatever bluetooth device it can find
    - from what is outputted get the address of the device with name 'CIRC BLE Beacon' 
        (address looks something like FC:69:47:7C:9D:A3)
    - 'bluetoothctl trust ADDRESS_FOUND_ABOVE'
    - 'bluetoothctl pair ADDRESS_FOUND_ABOVE'
    - 'bluetoothctl connect ADDRESS_FOUND_ABOVE'
    - the 'recieved value' (rssi) should then be published by the beacon and display in the terminal
    To get a (very) rough conversion to distance use the the getDistance() function with the rssi value.

    
    Notes (if another update gets the other functions working): 
    - bluepy is likely not already install and needs to be, to do that in the terminal run:
    'sudo apt-get install python3-pip libglib2.0-dev'
    'sudo pip3 install bluepy'
    - must be run as root
    """
    rospy.init_node("radio_beacon_dist")
    global pub 
    pub = rospy.Publisher('rssi_value', Int16, queue_size=1)
    discoverBLE()


def discoverBLE():
    """
    connect to the ble beacon and prints the rssi and distance to beacon onto the screen
    """
    # we search for the correct beacon since we do not know the address
    scanner = Scanner()
    devices = scanner.scan(2.0) #2.0 is the number of seconds it scans

    for dev in devices:
        dev
        print("Device %s (%s), RSSI=%d dB" % (dev.addr, dev.addrType, dev.rssi))
        for (adtype, desc, value) in dev.getScanData():
            print("  %s = %s" % (desc, value))


def requestBLE(address, name):
    """
    helper function which request data from the beacon
    """

    pass
    

def getDistance(rssi):
    """
    given the rssi (int) and known broadcasting device, we calculate the distance (roughly) to the radio beacon

    measuredRSSI is meant to be the rssi at 1 m, from link articles given for reference in the radio beaon task
    if seems -30 is the correct value but can't be sure 

    rssi < measuredRSSI (ex. -67) is further than one meter from beacon
    """
    N = 2.4 # estimated environmental factor should be between 2-4 and depends on heat, air quality, etc..
    measuredRSSI = -37 
    dist = math.pow(10.0, ((measuredRSSI - rssi) / (10 * N)))
    print(dist)


if __name__ == '__main__':
    #main()
    rssi = 0
    getDistance(rssi)
