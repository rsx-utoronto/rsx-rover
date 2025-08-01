#!/usr/bin/env python3

#Class for opening the science arduino and sending messages
import subprocess
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

board_name = "Arduino__www.arduino.cc__0042_334383935313517160E1"

class ScienceArduino(Node):
    def __init__(self, board_name):
        super().__init__('science_arduino_node')
        self.board_name = board_name
        self.port_name = self.find_port()
        self.sci_board = serial.Serial(self.port_name, baudrate=9600, timeout=1)
        # self.sci_pub = rospy.Publisher("/science_serial_data", String, queue_size=10)
        # self.sci_sub = rospy.Subscriber("/science_serial_control", String, self.board_write_callback)
        self.sci_pub = self.create_publisher(String, "/science_serial_data", 10)
        self.sci_sub = self.create_subscription(String, "/science_serial_control", self.board_write_callback, 10)
    
    def find_port(self):
        command = 'rover_ws/src/rsx-rover/scripts/utils/gen/find_usb.sh'

        output = subprocess.run(['bash', command], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # Get the output and errors from the script
        stdout = output.stdout.decode()

        # Split the output into lines
        lines = stdout.splitlines()
        
        sci_port = ''
        found_sci_port = False

        for line in lines:
            # Check for the USB camera device header
            if self.board_name in line:
              found_sci_port = True
            
            # If we are in the USB camera section, look for the device path
            if found_sci_port:
                if line.strip():  # If the line is not empty
                    line_parsed = line.split(' ')
                    sci_port = line_parsed[0]
                    break  # Stop after getting the first device path

        return sci_port

    def board_write_callback(self, msg):
        data = msg.data
        self.sci_board.write(bytes(data, 'utf-8'))
    
    def board_read(self):
        # Read line as bytes and decode to string first
        raw_data = self.sci_board.readline()
        
        # Decode bytes to string and split to remove newline
        if raw_data:
            # print(raw_data)
            data = raw_data.decode('utf-8').strip()  # strip() removes \n, \r etc.
            self.sci_pub.publish(data)


def main():
    rclpy.init(args=None)
    try:
        science_arduino = ScienceArduino(board_name)
        # Set the rate to 10 Hz
   
        while rclpy.ok():
            science_arduino.board_read()
            time.sleep(0.1)
        # rospy.spin()
        
    except rclpy.exceptions.ROSInterruptException:
        pass
    

if __name__ == "__main__":
    main()