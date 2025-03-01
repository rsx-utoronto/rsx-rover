
#!/usr/bin/env python3

#import rospy
#from rover.msg import StateMsg
#from std_msgs.msg import String
import time
import serial
import subprocess 
from std_msgs.msg import String
import rospy

#Make a subscriber in here instead of the State Machine file

#For Windows
def windows_get_led_port(led_name):
    #For linux
    # Run the v4l2-ctl command and capture the output
    command = "Get-WMIObject Win32_PnPEntity | Where-Object { $_.DeviceID -match '^USB' } | Select-Object DeviceID, Name"

    result = subprocess.run(["powershell", "-Command", command], capture_output=True, text=True)

    # Split the output into lines
    lines = result.stdout.splitlines()
    
    led_device = None
    found_led = False

    for line in lines:
        # Check for the led device header
        if led_name in line:
            found_led = True
            
        
        # If we are in the led section, look for the device path
        if found_led:
            led_device = line.split(' ') [0]
            break  # Stop after getting the first device path

    print(led_device)
    return led_device

#For Linux

def linux_get_led_port(led_name):
    # Run the v4l2-ctl command and capture the output
    result = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True, text=True)

    # Split the output into lines
    lines = result.stdout.splitlines()
    
    led_device = None
    found_led = False

    for line in lines:
        # Check for the USB camera device header
        if led_name in line:
            found_led = True
            continue
        
        # If we are in the USB camera section, look for the device path
        if found_led:
            if line.strip():  # If the line is not empty
                led_device = line.strip()
                break  # Stop after getting the first device path

    return led_device

def state_callback(msg):
  #seried_port = linux_get_led_port(led_name)
  #seried_port = windows_get_led_port(led_name)
  serial_port = "/dev/ttyUSB0" #Find out the seriel_port
  baud_rate = 9600
  ser = serial.Serial(serial_port, baud_rate, timeout=1)

  mode = msg.data
  print(mode)
  #global res
  #mode = msg.rover_mode stm32
  if mode == 'mission done':
    res = 'green\n' #How to get it to flash??
    for i in range(3):
      print(res)
      ser.open()
      ser.write(res.encode()) #Figure out the communication method
      ser.close()
      time.sleep(0.3) 
      ser.open()   
      ser.write(bytes([0])) #should reset the LED
      ser.flush() # Ensure it's sent before closing
      ser.close()
      time.sleep(0.3)
    
  elif mode == 'auto': 
    res = 'red\n'
    print(res)
    ser.open()
    ser.write(res.encode()) #Figure out the communication method
    ser.close() 
  #else:
  #  res = 'blue\n' 

def main():
   rospy.init_node('led_listener', anonymous=True)
   led_subscriber = rospy.Subscriber("led_light", String, state_callback)
   rospy.spin()


#Is this part needed?
#rospy.init_node('led_listener', anonymous=True)

# spin() simply keeps python from exiting until this node is stopped
#rospy.spin()

if __name__ == "__main__":
   #linux_get_led_port("COM3")
   main()

