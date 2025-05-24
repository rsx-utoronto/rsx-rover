#!/usr/bin/env python3

import time
import serial
import subprocess 
import serial.tools.list_ports
from std_msgs.msg import String
import rospy
import os
import rospkg

#For Windows
class LedLight():
    def __init__(self):
      self.led_sub = rospy.Subscriber("led_light", String, self.state_callback)
      self.board = None
      self.done_pre = False
      self.mode = "off"
      self.init_board()

    def list_all_ports(self):
      ports = [port.device for port in serial.tools.list_ports.comports()]
      return ports

    def windows_get_led_port(self):
      item_name = "USB\\VID_0483&PID_374B&MI_02\\6&348E67AE&0&0002" #Not updated

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
        # locate the find_usb.sh script in the rsx-rover package
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('rover')
        script_path = os.path.join(pkg_path, 'scripts/utils/gen/find_usb.sh')
        if not os.path.isfile(script_path):
            rospy.logerr(f"LED port finder script not found: {script_path}")
            return ''
        
        result = subprocess.run(['bash', script_path],
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE,
                                text=True)
        if result.returncode != 0:
            rospy.logerr(f"Error running {script_path}: {result.stderr.strip()}")
            return ''
        stdout = result.stdout
        print("stdout", stdout)
        # Split the output into lines
        lines = stdout.splitlines()
        
        led_port = ''

        led_device = '1a86_USB2.0-Serial'
        found_led = False

        for line in lines:
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

    def init_board(self):
      serial_port =  self.linux_get_led_port().strip() #only works when accessing with sudo
      print("Serial Port:", serial_port)
      # serial_port = "/dev/ttyUSB5" #Find out the serial_port
      self.board = serial.Serial(port=serial_port, baudrate=115200, timeout=1)
      rospy.sleep(2)
      self.board.write(bytes('blue\n', 'utf-8'))
      print("I'm initialized!")
      
      
    def state_callback(self, msg):
      mode = msg.data.strip()
      # mode = msg.data
      #print(mode)
      #global res
      #mode = msg.rover_mode stm32
      if mode == 'mission done':
        res = 'green\n' # 'green' not working yet
        #for i in range(3):
          #if i > 0:
            #self.board.open()
        self.board.write(bytes(res, 'utf-8')) 
          #board.close() #dont need a while loop because board does it automatically!
          #time.sleep(0.3) 
          #board.open()   
          #board.write(bytes('off\n', 'utf-8'))
          #board.flush() # Ensure it's sent before closing
          #board.close()
          #time.sleep(0.3)
        
      elif mode == 'auto': 
        res = 'red\n'
        print(res)
        x = self.board.write(bytes(res, 'utf-8'))
        print(x)
      elif mode == "manual":
        res = 'blue\n' 
        print(res)
        x = self.board.write(bytes(res, 'utf-8'))
        print(x)
      else:
        res = 'off\n' 
        print(res)
        x = self.board.write(bytes(res, 'utf-8'))
        print(x)
        

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('led_listener', anonymous=True)
    try:
        led = LedLight()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    