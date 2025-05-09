#!/usr/bin/env python3

#import rospy
#from rover.msg import StateMsg
#from std_msgs.msg import String
import time
import serial
import subprocess 
import serial.tools.list_ports
from std_msgs.msg import String
import rospy
#from std_msgs.msg import String
#import rospy

#Make a subscriber in here instead of the State Machine file

#For Windows

class LedLight():
   #rospy.init_node('led_listener', anonymous=True) #is it needed - delete if not needed 
    def __init__(self):
      self.led_sub = rospy.Subscriber("led_light", String, self.state_callback)

    def list_all_ports(self):
      ports = [port.device for port in serial.tools.list_ports.comports()]
      return ports

    def windows_get_led_port(self):
      item_name = "USB\\VID_0483&PID_374B&MI_02\\6&348E67AE&0&0002"

      command = "Get-WMIObject Win32_SerialPort"

      result = subprocess.run(["powershell", "-Command", command], capture_output=True, text=True)

      connected_items = result.stdout.splitlines()
      i = 0
      port_name = ""
      while (i < len(connected_items)):
        if connected_items[i] == "":
          pass
        if item_name in connected_items[i]:
          port_id_list = connected_items[i-11].split(": ")
          port_name = port_id_list[len(port_id_list)-1]
          break
        i+=1

      ports = self.list_all_ports()
      if len(ports) == 0:
        print("No port found. Exiting.")
        import sys
        sys.exit(0)
      elif len(ports) > 1:
        print("There are multiple ports connected")
        for port in ports:
          if port == port_name:
              print(port)
              return port
        
      else:
        print("One port found")
        port = ports[0]
        if port == port_name:
              print(port)
              return port


    def linux_get_led_port(self):
        # Run the v4l2-ctl command and capture the output
        #journalctl -k | grep -i usb

        command = 'rover_ws/src/rsx-rover/scripts/utils/gen/find_usb.sh'

        output = subprocess.run(['bash', command], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    # Get the output and errors from the script
        stdout = output.stdout.decode()

        # Split the output into lines
        lines = stdout.splitlines()
        
        led_port = ''
        led_device = 'STMicroelectronics_STM32_STLink_0672FF575487884867140825'
        found_led = False

        for line in lines:
            print(line)
            # Check for the USB camera device header
            if led_device in line:
              found_led = True
              print("Found LED device header:", led_device)
            
            # If we are in the USB camera section, look for the device path
            if found_led:
                if line.strip():  # If the line is not empty
                    line_parsed = line.split(' ')
                    led_port = line_parsed[0]
                    print("Found LED port:", led_port)
                    break  # Stop after getting the first device path

        return led_port #led_device

    def state_callback(self, msg):
      mode = msg.data.strip()
      serial_port =  self.linux_get_led_port().strip() #only works when accessing with sudo
      print("Serial Port:", serial_port)
      #serial_port = "/dev/ttyUSB0" #Find out the seriel_port
      board = serial.Serial(port=serial_port, baudrate=115200, timeout=1)

      # mode = msg.data
      #print(mode)
      #global res
      #mode = msg.rover_mode stm32
      if mode == 'mission done':
        res = 'green' # 'green' not working yet
        for i in range(3):
          if i > 0:
            board.open()
          board.write(bytes(res, 'utf-8')) 
          board.close()
          time.sleep(0.3) 
          board.open()   
          board.write(bytes('off\n', 'utf-8'))
          board.flush() # Ensure it's sent before closing
          board.close()
          time.sleep(0.3)
        
      elif mode == 'auto': 
        res = 'red'
        print(res)
        x = board.write(bytes(res, 'utf-8'))
        print(x)
      else:
        res = 'blue' 
        print(res)
        x = board.write(bytes(res, 'utf-8'))
        print(x)
        

# spin() simply keeps python from exiting until this node is stopped
#rospy.spin()

#if __name__ == "__main__":
#   main()

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('led_listener', anonymous=True)
    try:
        led = LedLight()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    