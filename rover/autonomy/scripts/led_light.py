
#!/usr/bin/env python3

#import rospy
#from rover.msg import StateMsg
#from std_msgs.msg import String
import time
import serial
import subprocess 
import serial.tools.list_ports
#from std_msgs.msg import String
#import rospy

#Make a subscriber in here instead of the State Machine file

#For Windows

def list_all_ports():
	ports = [port.device for port in serial.tools.list_ports.comports()]
	return ports

def windows_get_led_port():
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

  ports = list_all_ports()
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

def state_callback(mode):
  #seried_port = linux_get_led_port(led_name)
  serial_port = windows_get_led_port()
  #serial_port = "/dev/ttyUSB0" #Find out the seriel_port
  board = serial.Serial(port=serial_port, baudrate=115200, timeout=1)

  # mode = msg.data
#   print(mode)
  #global res
  #mode = msg.rover_mode stm32
  if mode == 'mission done':
    res = 'green\n' #How to get it to flash??
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
    res = 'red\n'
    print(res)
    board.write(bytes(res, 'utf-8'))
  else:
    res = 'blue\n' 
    print(res)
    board.write(bytes(res, 'utf-8'))

def main():
   #rospy.init_node('led_listener', anonymous=True)
   #led_subscriber = rospy.Subscriber("led_light", String, state_callback)
   state_callback('auto')
   #rospy.spin()


#Is this part needed?
#rospy.init_node('led_listener', anonymous=True)

# spin() simply keeps python from exiting until this node is stopped
#rospy.spin()

if __name__ == "__main__":
   main()
