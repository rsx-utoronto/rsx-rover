#!/usr/bin/env python3

import serial as ser
import time
import tkinter as tk
import os

BUTTONWIDTH = 5
BUTTONHEIGHT = 2

BAUDRATE = 9600
PORT     = 'COM7'
PERIOD   = 5 # Assuming we are getting 1 output over serial every second

class Serial_Port:
    """
    Making a class to deal with serial ports
    """

    def __init__(self, device_name : str) -> None:
        """
        (String) -> (None)

        Initializing the class by setting device variables. 
        Do an initial attempt at connection right at the start.
        """

        # Setting up some variables
        self.device_port = None
        self.device_name = device_name

    def open_device_port(self, baudrate= BAUDRATE):
        """
        (None) -> (None)

        Establishes the connection between the code and the
        serial port.
        """

        #initializing serial port with baudrate 9600
        self.device_port = ser.Serial(self.device_name, baudrate, timeout= 2)
    
    def send_bytes(self, data : str):
        """
        (String) -> (None)

        Send bytes over the serial port
        """

        # Checking if device port is valid
        if self.device_port:

            # Write data value to device port
            self.device_port.write(data.encode('utf-8'))

        else:
            print("\nPort not connected")
    
    def read_bytes(self, endline : str = '\n'): #endline : str = '\n'
        if self.device_port:
            # data = self.device_port.readlines()
            # specificline = data[i]
            # Read data value coming to the device port
            data = self.device_port.read_until(expected= endline.encode('utf-8'))
            return str(data, encoding= 'utf-8')
        
        else:
            print("\nPort not connected")
            return ""
    def close_device_port(self):
        """
        (None) -> (None)

        Closes device port 
        """

        # Checking if device port is valid
        if self.device_port:

            # Close the device port and set device_port global back to 0
            self.device_port.close()
            self.device_port = None
        
        else:
            print("\nPort not connected\n")

#send data over serial bus, and print incoming data
def send_data(letter : str, connection : Serial_Port, expected_result : str = None):
    connection.send_bytes(letter)
    #change the bit number to 16 if you remove the print statements in the arduino code
    TIMEOUT = time.time()
    while True:
        if not expected_result:
            break
        data = connection.read_bytes()
        if expected_result in data:
            print(data)
            break
        elif time.time() - TIMEOUT > 5:
            print("Timeout")
            break

    print(connection.read_bytes())

def get_temp_humidity(connection : Serial_Port):
    send_data("&", connection)
    # Creating a txt file to store the data
    data_file = open("Temperatureandhumidity.txt", "a")

    # Read temperature value for the time period (assuming a reading is sent every second)
    for i in range(PERIOD):
        TIMEOUT = time.time()
        while True:
            temperature = connection.read_bytes() #endline= 'Ft
            print(temperature)
            if "Temp" in temperature:
                break
            elif time.time()-TIMEOUT >2:
                print("Timeout")
                break

        connection.device_port.reset_input_buffer()
        # if ('Sensor' in temperature):
        data_file.write(temperature[ : ])
        print(temperature[ : ])
        # else:
            # i -= 1

    data_file.close()

def get_PMT(connection : Serial_Port):
    send_data("p", connection)
    # Creating a txt file to store the data
    data_file = open("PMT.txt", "a")

    # Read temperature value for the time period (assuming a reading is sent every second)
    for i in range(PERIOD):
        TIMEOUT = time.time()
        while True:
            voltage = connection.read_bytes()
            if "lkj" in voltage:
                break
            elif time.time()-TIMEOUT>2:
                print("Timeout")
                break
        connection.device_port.reset_input_buffer()
        data_file.write(voltage[ : ])
        print(voltage[ : ])

    data_file.close()


m=tk.Tk()  #main window
m.title('Science Team Drill Controls')  #window title

#connect to arduino
# Create Serial Connection object
connection = Serial_Port(PORT)

try:
    # Open the serial connectioon
    connection.open_device_port(baudrate= BAUDRATE)
    time.sleep(0.5)
    
except ser.serialutil.SerialException:
    print("Arduino not connected")


#Emergency
estop = tk.Button(m, text='E-STOP', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="red",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('s', connection))
reset = tk.Button(m, text='Reset', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="red",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('z', connection))

##change delays
delay_100 = tk.Button(m, text='D: 100', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="orange",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('1', connection))
delay_500 = tk.Button(m, text='D: 500', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="orange",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('2', connection))
delay_1000 = tk.Button(m, text='D: 1000', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="orange",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('3', connection))
delay_3000 = tk.Button(m, text='D: 3000', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="orange",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('4', connection))

#Drill 1 Up
drill_up = tk.Button(m, text='Drill Up', 
                       width=BUTTONWIDTH*2, 
                       height= BUTTONHEIGHT, 
                       bg="yellow",
                       font= ('Helvetica 20 bold'),
                       command= lambda:send_data('u', connection))
#Drill 1 Down
drill_down = tk.Button(m, text='Drill Down', 
                         width=BUTTONWIDTH*2,
                         height= BUTTONHEIGHT, 
                         bg="yellow",
                         font= ('Helvetica 20 bold'),
                         command= lambda:send_data('d', connection))
#Drill 1 FWD
drill_FWD = tk.Button(m, text='Drill CW', #this is usually what we want
                        width=BUTTONWIDTH*2, 
                        height= BUTTONHEIGHT,
                        bg="yellow",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('c', connection))
#Drill 1 REV
drill_REV = tk.Button(m, text='Drill CCW', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="yellow",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('x', connection))
#   
#attach bit
drill_attachbit = tk.Button(m, text='Attach Bit', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="yellow",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('f', connection))

#remove bit
drill_removebit = tk.Button(m, text='Remove Bit', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="yellow",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('g', connection))

# ##rotate changer
surface1 = tk.Button(m, text='Surface 1', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('q', connection))
surface2 = tk.Button(m, text='Surface 2', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('w', connection))
narrow = tk.Button(m, text='Narrow', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('e', connection))
deep = tk.Button(m, text='Deep', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('r', connection))
camera = tk.Button(m, text='Camera', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('t', connection))
sensor = tk.Button(m, text='Sensor', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('y', connection))
lastpos = tk.Button(m, text='Empty', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('a', connection))
hole = tk.Button(m, text='Hole', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('h', connection))


##Small steps for bit changer
tinycw = tk.Button(m, text='Bit Changer Small CW', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('k', connection))
tinyccw = tk.Button(m, text='Bit Changer Small CCW', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('l', connection))
resetbc = tk.Button(m, text='Reset Bit Changer', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('o', connection))

#Temperature sensor
# tempon = tk.Button(m, text='Temp Sensor On', 
#                         width=BUTTONWIDTH,
#                         height= BUTTONHEIGHT, 
#                         bg="green",
#                         font= ('Helvetica 20 bold'),
#                         command= lambda:send_data('&', connection))
tempsensor = tk.Button(m, text='Temp and Humidity Data', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="green",
                        font= ('Helvetica 20 bold'),
                        command= lambda:get_temp_humidity(connection))

# PMT = tk.Button(m, text='PMT On/Off', 
#                         width=BUTTONWIDTH,
#                         height= BUTTONHEIGHT, 
#                         bg="green",
#                         font= ('Helvetica 20 bold'),
#                         command= lambda:send_data('p', connection))
PMTon = tk.Button(m, text='PMT Light On', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="green",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('^', connection))
PMToff = tk.Button(m, text='PMT Light Off', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="green",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('%', connection))
PMTdata = tk.Button(m, text='PMT Data', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="green",
                        font= ('Helvetica 20 bold'),
                        command= lambda:get_PMT(connection))

##analysis
# testtube = tk.Button(m, text='Test Tube', 
#                         width=BUTTONWIDTH*2,
#                         height= BUTTONHEIGHT, 
#                         bg="lightblue",
#                         font= ('Helvetica 20 bold'),
#                         command= lambda:send_data(b't'))
valve1 = tk.Button(m, text='Valve 1', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="lightblue",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('v', connection))
valve2 = tk.Button(m, text='Valve 2', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="lightblue",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('b', connection))
valve3 = tk.Button(m, text='Valve 3', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="lightblue",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('n', connection))
valve4 = tk.Button(m, text='Valve 4', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="lightblue",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('m', connection))
pump1 = tk.Button(m, text='Pump 1', 
                        width=BUTTONWIDTH*4,
                        height= BUTTONHEIGHT, 
                        bg="lightblue",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('i', connection))
pump2 = tk.Button(m, text='Pump 2', 
                        width=BUTTONWIDTH*4,
                        height= BUTTONHEIGHT, 
                        bg="lightblue",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('j', connection))

## analysis module
TTCW = tk.Button(m, text='Test Tube CW', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="lightgreen",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('>', connection))
TTCCW = tk.Button(m, text='Test Tube CCW', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="lightgreen",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('<', connection))
smallCW = tk.Button(m, text='Test Tube Small CW', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="lightgreen",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(')', connection))
smallCCW = tk.Button(m, text='Test Tube Small CCW', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="lightgreen",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('(', connection))

lighton = tk.Button(m, text='Light On', ##]
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="white",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(']', connection))
lightoff = tk.Button(m, text='Light Off', ##]
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="white",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('[', connection))

##PMT
shutter_open = tk.Button(m, text='Open Shutter', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(',', connection))
shutter_close = tk.Button(m, text='Close Shutter', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data('.', connection))


# Specify Grid
tk.Grid.columnconfigure(m, index = 0, weight = 1)
tk.Grid.columnconfigure(m, index = 1, weight = 1)
tk.Grid.columnconfigure(m, index = 2, weight = 1)
tk.Grid.columnconfigure(m, index = 3, weight = 1)
tk.Grid.columnconfigure(m, index = 4, weight = 1)
tk.Grid.columnconfigure(m, index = 5, weight = 1)
tk.Grid.columnconfigure(m, index = 6, weight = 1)
tk.Grid.columnconfigure(m, index = 7, weight = 1)

tk.Grid.rowconfigure(m, 0,weight = 1)
tk.Grid.rowconfigure(m, 1,weight = 1)
tk.Grid.rowconfigure(m, 2,weight = 1)
tk.Grid.rowconfigure(m, 3,weight = 1)
tk.Grid.rowconfigure(m, 4,weight = 1)
tk.Grid.rowconfigure(m, 5,weight = 1)
tk.Grid.rowconfigure(m, 6,weight = 1)
tk.Grid.rowconfigure(m, 7,weight = 1)
tk.Grid.rowconfigure(m, 8,weight = 1)
tk.Grid.rowconfigure(m, 9,weight = 1)
tk.Grid.rowconfigure(m, 10,weight = 1)

#place buttons
estop.grid(row=0,column=0,columnspan=4,sticky="NSEW")
reset.grid(row=0,column=4,columnspan=4,sticky="NSEW")

delay_100.grid(row=1,column=0,columnspan=2,sticky="NSEW")
delay_500.grid(row=1,column=2,columnspan=2,sticky="NSEW")
delay_1000.grid(row=1,column=4,columnspan=2,sticky="NSEW")
delay_3000.grid(row=1,column=6,columnspan=2,sticky="NSEW")

drill_up.grid(row=2,column=0,columnspan=2,sticky="NSEW")
drill_down.grid(row=2,column=2,columnspan=2,sticky="NSEW")
drill_FWD.grid(row=2,column=4,columnspan=2,sticky="NSEW")
drill_REV.grid(row=2,column=6,columnspan=2,sticky="NSEW")

drill_attachbit.grid(row=3,column=0,columnspan=4,sticky="NSEW")
drill_removebit.grid(row=3,column=4,columnspan=4,sticky="NSEW")

surface1.grid(row=4,column=0,columnspan=1,sticky="NSEW")
surface2.grid(row=4,column=1,columnspan=1,sticky="NSEW")
narrow.grid(row=4,column=2,columnspan=1,sticky="NSEW")
deep.grid(row=4,column=3,columnspan=1,sticky="NSEW")
camera.grid(row=4,column=4,columnspan=1,sticky="NSEW")
sensor.grid(row=4,column=5,columnspan=1,sticky="NSEW")
lastpos.grid(row=4,column=6,columnspan=1,sticky="NSEW")
hole.grid(row=4,column=7,columnspan=1,sticky="NSEW")

tinycw.grid(row=5,column=0,columnspan=2,sticky="NSEW")
tinyccw.grid(row=5,column=2,columnspan=2,sticky="NSEW")
resetbc.grid(row=5,column=4,columnspan=4,sticky="NSEW")

#tempon.grid(row=6,column=0,columnspan=2,sticky="NSEW")
tempsensor.grid(row=6,column=0,columnspan=2,sticky="NSEW")
PMTon.grid(row=6,column=2,columnspan=2,sticky="NSEW")
PMToff.grid(row=6,column=4,columnspan=2,sticky="NSEW")
#PMT.grid(row=6,column=4,columnspan=2,sticky="NSEW")
PMTdata.grid(row=6,column=6,columnspan=2,sticky="NSEW")

valve1.grid(row=7,column=0,columnspan=2,sticky="NSEW")
valve2.grid(row=7,column=2,columnspan=2,sticky="NSEW")
valve3.grid(row=7,column=4,columnspan=2,sticky="NSEW")
valve4.grid(row=7,column=6,columnspan=2,sticky="NSEW")

pump1.grid(row=8,column=0,columnspan=4,sticky="NSEW")
pump2.grid(row=8,column=4,columnspan=4,sticky="NSEW")

TTCW.grid(row=9,column=0,columnspan=2,sticky="NSEW")
TTCCW.grid(row=9,column=2,columnspan=2,sticky="NSEW")
smallCW.grid(row=9,column=4,columnspan=2,sticky="NSEW")
smallCCW.grid(row=9,column=6,columnspan=2,sticky="NSEW")

lighton.grid(row=10,column=0,columnspan=2,sticky="NSEW")
lightoff.grid(row=10,column=2,columnspan=2,sticky="NSEW")
shutter_open.grid(row=10,column=4,columnspan=2,sticky="NSEW")
shutter_close.grid(row=10,column=6,columnspan=2,sticky="NSEW")

m.mainloop()
