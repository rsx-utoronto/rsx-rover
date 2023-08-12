#!/usr/bin/env python3
import requests
import os
import rospy
import html


def main():
    """
    Currently we hardcode if the task search or rescue (see var below), then connect to the corresponding
    open ssid (wifi). Lastly accesses the webpage to get the gps location (of the astronuat or reactor).
    """
    rospy.init_node("gps_loc_from_wifi")
    task = 'search'

    while not rospy.is_shutdown():
        if task == 'search' or task == 'rescue':
            if connectToWIFI(task):
                getGPSLocation()
        rospy.sleep(1) # if it didn't connect the first time wait before trying again


def connectToWIFI(curr):
    """
    curr is either search or rescue
    This function scans (using the linux command iwlist) for wifi hosts, and if it finds the correct one for the 
    current task then it trys to join it.
    """
    if curr == "search":
        ssid = "Isaac_Asimov"
    else:
        ssid = "Reactor"
    command = """sudo nmcli device wifi connect {}"""
    try:
        os.system(command.format(ssid))
    except:
        raise
    

def getGPSLocation():
    """
    access the webpage then extract the gps location from the html
    """
    url = "http://10.10.11.1:80"
    req = requests.get(url)
    ht = html.unescape(req.text)
    txt = ht.split("Location: ")[1] # removes everything before Location:
    txt2 = txt.split("W")[0] # the gps location as show in example https://github.com/canspacetech/CIRC-2023-Night-Task-HW-SW/blob/main/Simulation/GPS%20Beacon%20Webpage%20Example/Beacon%20Webpage%20with%20GPS%20Example%2010.10.10.1.html
    print("The given gps coordinates")
    print(txt2 + "W")
    lat, long = txt2.replace("N", "").split(", ")
    print("Lat: " + lat)
    print("Long:" + long)


if __name__ == '__main__':
    main()
