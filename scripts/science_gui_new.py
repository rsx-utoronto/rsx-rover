#!/usr/bin/env python3 

import serial as ser
import time
import tkinter as tk

BUTTONWIDTH = 5
BUTTONHEIGHT = 2

#send data over serial bus, and print incoming data
def send_data(letter):
    ser.write(letter)
    #change the bit number to 16 if you remove the print statements in the arduino code
    print(ser.read(26)) 


m=tk.Tk()  #main window
m.title('Science Team Drill Controls')  #window title

#connect to arduino
try:
    ser = ser.Serial("COM4", 9600)
    time.sleep(0.5)
except ser.serialutil.SerialException:
    print("Arduino not connected")

##change delays
delay_100 = tk.Button(m, text='D: 100', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="orange",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'z100'))
delay_500 = tk.Button(m, text='D: 500', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="orange",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'z500'))
delay_1000 = tk.Button(m, text='D: 1000', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="orange",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'z1000'))
delay_3000 = tk.Button(m, text='D: 3000', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="orange",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'z3000'))

#Drill 1 Up
drill_up = tk.Button(m, text='Drill Up', 
                       width=BUTTONWIDTH*2, 
                       height= BUTTONHEIGHT, 
                       bg="yellow",
                       font= ('Helvetica 20 bold'),
                       command= lambda:send_data(b'u'))
#Drill 1 Down
drill_down = tk.Button(m, text='Drill Down', 
                         width=BUTTONWIDTH*2,
                         height= BUTTONHEIGHT, 
                         bg="yellow",
                         font= ('Helvetica 20 bold'),
                         command= lambda:send_data(b'd'))
#Drill 1 FWD
drill_FWD = tk.Button(m, text='Drill CW', #this is usually what we want
                        width=BUTTONWIDTH*2, 
                        height= BUTTONHEIGHT,
                        bg="yellow",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'c'))
#Drill 1 REV
drill_REV = tk.Button(m, text='Drill CCW', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="yellow",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'x'))
#   
#attach bit
drill_attachbit = tk.Button(m, text='Attach Bit', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="yellow",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'f'))

#remove bit
drill_removebit = tk.Button(m, text='Remove Bit', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="yellow",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'g'))

# ##rotate changer
surface1 = tk.Button(m, text='Surface 1', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'ow'))
surface2 = tk.Button(m, text='Surface 2', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'ow'))
narrow = tk.Button(m, text='Narrow', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'ow'))
deep = tk.Button(m, text='Deep', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'ow'))
narrow = tk.Button(m, text='Narrow', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'ow'))
camera = tk.Button(m, text='Camera', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'ow'))
sensor = tk.Button(m, text='Sensor', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'ow'))
hole = tk.Button(m, text='Hole', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'h'))

##Small steps for bit changer
tinycw = tk.Button(m, text='Small CW', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'k'))
tinyccw = tk.Button(m, text='Small CCW', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'l'))

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
                        command= lambda:send_data(b'v'))
valve2 = tk.Button(m, text='Valve 2', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="lightblue",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'b'))
valve3 = tk.Button(m, text='Valve 3', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="lightblue",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'n'))
valve4 = tk.Button(m, text='Valve 4', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="lightblue",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'm'))
pump1 = tk.Button(m, text='Pump 1', 
                        width=BUTTONWIDTH*4,
                        height= BUTTONHEIGHT, 
                        bg="lightblue",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'i'))
pump2 = tk.Button(m, text='Pump 2', 
                        width=BUTTONWIDTH*4,
                        height= BUTTONHEIGHT, 
                        bg="lightblue",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'j'))

## analysis module
TTCW = tk.Button(m, text='TT CW', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="lightgreen",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'j'))
TTCCW = tk.Button(m, text='TT CCW', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="lightgreen",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'j'))
smallCW = tk.Button(m, text='Small CW', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="lightgreen",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'j'))
smallCCW = tk.Button(m, text='Small CCW', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="lightgreen",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'j'))

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
#tk.Grid.rowconfigure(m, 8,weight = 1)

#place buttons
delay_100.grid(row=0,column=0,columnspan=2,sticky="NSEW")
delay_500.grid(row=0,column=2,columnspan=2,sticky="NSEW")
delay_1000.grid(row=0,column=4,columnspan=2,sticky="NSEW")
delay_3000.grid(row=0,column=6,columnspan=2,sticky="NSEW")

drill_up.grid(row=1,column=0,columnspan=2,sticky="NSEW")
drill_down.grid(row=1,column=2,columnspan=2,sticky="NSEW")
drill_FWD.grid(row=1,column=4,columnspan=2,sticky="NSEW")
drill_REV.grid(row=1,column=6,columnspan=2,sticky="NSEW")

drill_attachbit.grid(row=2,column=0,columnspan=4,sticky="NSEW")
drill_removebit.grid(row=2,column=4,columnspan=4,sticky="NSEW")

surface1.grid(row=3,column=0,columnspan=1,sticky="NSEW")
surface2.grid(row=3,column=1,columnspan=1,sticky="NSEW")
narrow.grid(row=3,column=2,columnspan=1,sticky="NSEW")
deep.grid(row=3,column=3,columnspan=1,sticky="NSEW")
camera.grid(row=3,column=4,columnspan=1,sticky="NSEW")
sensor.grid(row=3,column=5,columnspan=1,sticky="NSEW")
hole.grid(row=3,column=6,columnspan=2,sticky="NSEW")

tinycw.grid(row=4,column=0,columnspan=4,sticky="NSEW")
tinyccw.grid(row=4,column=4,columnspan=4,sticky="NSEW")

valve1.grid(row=5,column=0,columnspan=2,sticky="NSEW")
valve2.grid(row=5,column=2,columnspan=2,sticky="NSEW")
valve3.grid(row=5,column=4,columnspan=2,sticky="NSEW")
valve4.grid(row=5,column=6,columnspan=2,sticky="NSEW")

pump1.grid(row=6,column=0,columnspan=4,sticky="NSEW")
pump2.grid(row=6,column=4,columnspan=4,sticky="NSEW")

TTCW.grid(row=7,column=0,columnspan=2,sticky="NSEW")
TTCCW.grid(row=7,column=2,columnspan=2,sticky="NSEW")
smallCW.grid(row=7,column=4,columnspan=2,sticky="NSEW")
smallCCW.grid(row=7,column=6,columnspan=2,sticky="NSEW")

m.mainloop()
